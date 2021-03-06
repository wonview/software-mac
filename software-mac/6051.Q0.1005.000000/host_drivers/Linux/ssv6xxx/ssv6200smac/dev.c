#include <linux/nl80211.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <net/mac80211.h>
#include <ssv6200.h>
#include "lib.h"
#include "ssv_rc.h"
#include "ssv_ht_rc.h"
#include "dev.h"
#include "ap.h"
#include "init.h"
#include "p2p.h"


#ifdef MULTI_THREAD_ENCRYPT
#include <linux/freezer.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
#include "linux_3_0_0.h"
#endif

#ifdef CONFIG_SSV6XXX_DEBUGFS
#include "ssv6xxx_debugfs.h"
#endif // CONFIG_SSV6XXX_DEBUGFS

//#define IRQ_PROC_RX_DATA
#define NO_USE_RXQ_LOCK

#ifndef WLAN_CIPHER_SUITE_SMS4
#define WLAN_CIPHER_SUITE_SMS4          0x00147201
#endif
static u16 bits_per_symbol[][2] =
{
    /* 20MHz 40MHz */
    {    26,   54 },     /*  0: BPSK */
    {    52,  108 },     /*  1: QPSK 1/2 */
    {    78,  162 },     /*  2: QPSK 3/4 */
    {   104,  216 },     /*  3: 16-QAM 1/2 */
    {   156,  324 },     /*  4: 16-QAM 3/4 */
    {   208,  432 },     /*  5: 64-QAM 2/3 */
    {   234,  486 },     /*  6: 64-QAM 3/4 */
    {   260,  540 },     /*  7: 64-QAM 5/6 */
};


struct ssv6xxx_calib_table {
    u16 channel_id;
    u32 rf_ctrl_N;
    u32 rf_ctrl_F;
    u16 rf_precision_default;
};


static void _process_rx_q (struct ssv_softc *sc, struct sk_buff_head *rx_q, spinlock_t *rx_q_lock);
static u32 _process_tx_done (struct ssv_softc *sc);

#ifdef MULTI_THREAD_ENCRYPT
struct list_head encrypt_task_head;

unsigned int skb_queue_len_safe(struct sk_buff_head *list) 
{ 
    unsigned long flags;
    unsigned int ret = 0;
    spin_lock_irqsave(&list->lock, flags);
    ret = skb_queue_len(list);
    spin_unlock_irqrestore(&list->lock, flags);
    return ret;
}  
#endif

#if 1
void _ssv6xxx_hexdump(const char *title, const u8 *buf,
                             size_t len)
{
    size_t i;
    printk("%s - hexdump(len=%lu):\n", title, (unsigned long) len);
    if (buf == NULL) {
        printk(" [NULL]");
    }else{
        for (i = 0; i < len; i++){
            
            printk(" %02x", buf[i]);
            if((i+1)%16 ==0)
                printk("\n");
        }
    }
    printk("\n-----------------------------\n");
}

#endif


void ssv6xxx_txbuf_free_skb(struct sk_buff *skb, void *args)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
    struct ssv_softc *sc = (struct ssv_softc *)args;
#endif

    if (!skb)
        return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
    ieee80211_free_txskb(sc->hw, skb);
#else
    dev_kfree_skb_any(skb);
#endif

}


//=====>ADR_CH0_TRIG_1
#define ADDRESS_OFFSET            16                    //16
#define HW_ID_OFFSET              7
//=====>

//=====>ADR_MCU_STATUS
#define CH0_FULL_MASK             CH0_FULL_MSK              //0x00000001
//=====>

#define MAX_FAIL_COUNT          100
#define MAX_RETRY_COUNT         20

inline bool ssv6xxx_mcu_input_full(struct ssv_softc *sc)
{
    u32 regval=0;
    SMAC_REG_READ(sc->sh, ADR_MCU_STATUS, &regval);
    return CH0_FULL_MASK&regval;
}

u32 ssv6xxx_pbuf_alloc(struct ssv_softc *sc, int size, int type)
{
    u32 regval, pad;
    int cnt = MAX_RETRY_COUNT;
    int page_cnt = (size + ((1 << HW_MMU_PAGE_SHIFT) - 1)) >> HW_MMU_PAGE_SHIFT;

    regval = 0;

    mutex_lock(&sc->mem_mutex);
    
    //brust could be dividen by 4
    pad = size%4;
    size += pad;

    do{
        //printk("[A] ssv6xxx_pbuf_alloc\n");

        SMAC_REG_WRITE(sc->sh, ADR_WR_ALC, (size | (type << 16)));
        SMAC_REG_READ(sc->sh, ADR_WR_ALC, &regval);
        
        if (regval == 0) {
            cnt--;
            msleep(1);
        }
        else
            break;
                
    } while (cnt);

    // If TX buffer is allocated, AMPDU maximum size m
    if (type == TX_BUF)
    {
        sc->sh->tx_page_available -= page_cnt;
        sc->sh->page_count[PACKET_ADDR_2_ID(regval)] = page_cnt;
    }

    mutex_unlock(&sc->mem_mutex);

    if (regval == 0)
        dev_err(sc->dev, "Failed to allocate packet buffer of %d bytes in %d type.",
                size, type);
    else
    {
        dev_info(sc->dev, "Allocated %d type packet buffer of size %d (%d) at address %x.\n",
                 type, size, page_cnt, regval);
    }
    
    return regval;
}

bool ssv6xxx_pbuf_free(struct ssv_softc *sc, u32 pbuf_addr)
{
    u32  regval=0;
    u16  failCount=0;
    u8  *p_tx_page_cnt = &sc->sh->page_count[PACKET_ADDR_2_ID(pbuf_addr)];
    
    while (ssv6xxx_mcu_input_full(sc))
    {
        if (failCount++ < 1000) continue;
            printk("=============>ERROR!!MAILBOX Block[%d]\n", failCount);
            return false;
    } //Wait until input queue of cho is not full.

    mutex_lock(&sc->mem_mutex);

    // {HWID[3:0], PKTID[6:0]}
    regval = ((M_ENG_TRASH_CAN << HW_ID_OFFSET) |(pbuf_addr >> ADDRESS_OFFSET));
    
    printk("[A] ssv6xxx_pbuf_free addr[%08x][%x]\n", pbuf_addr, regval);
    SMAC_REG_WRITE(sc->sh, ADR_CH0_TRIG_1, regval);

    if (*p_tx_page_cnt)
    {
        sc->sh->tx_page_available += *p_tx_page_cnt;
        *p_tx_page_cnt = 0;
    }

    mutex_unlock(&sc->mem_mutex);
    
    return true;
}



#ifdef CONFIG_SSV_CABRIO_A
static const struct ssv6xxx_calib_table vt_tbl[] =
{
    /* Table for Cabrio A  40M */
    {  1, 0xf1, 0x333333, 3859},
    {  2, 0xf1, 0xB33333, 3867},
    {  3, 0xf2, 0x333333, 3875},
    {  4, 0xf2, 0xB33333, 3883},
    {  5, 0xf3, 0x333333, 3891},
    {  6, 0xf3, 0xB33333, 3899},
    {  7, 0xf4, 0x333333, 3907},
    {  8, 0xf4, 0xB33333, 3915},
    {  9, 0xf5, 0x333333, 3923},
    { 10, 0xf5, 0xB33333, 3931},
    { 11, 0xf6, 0x333333, 3939},
    { 12, 0xf6, 0xB33333, 3947},
    { 13, 0xf7, 0x333333, 3955},
    { 14, 0xf8, 0x666666, 3974},
};

int ssv6xxx_set_channel(struct ssv_softc *sc, int ch)
{
    int retry_cnt, fail_cnt=0;
    struct ssv_hw *sh=sc->sh;
    u32 regval;
    int ret = 0;

    int  chidx;
    bool chidx_vld = 0;

    for(chidx = 0; chidx < (sizeof(vt_tbl)/sizeof(vt_tbl[0])); chidx++) {
        if (vt_tbl[chidx].channel_id == ch) {
            chidx_vld = 1;
            break;
        }
    }

    if (chidx_vld == 0) {
        printk("%s(): fail! channel_id not found in vt_tbl\n", __FUNCTION__);
        return -1;
    }

    do {
        //this set the clock width of spi to phy
        //from 6 --> 5
        if ((ret = SMAC_REG_READ(sh, ADR_SPI_TO_PHY_PARAM1, &regval)) != 0) break;
        if ((ret = SMAC_REG_WRITE(sh,ADR_SPI_TO_PHY_PARAM1,(regval&~0xffff)|3)) != 0) break;
        
        ssv6xxx_rf_disable(sc->sh);
        
        /* step 1: SET_CBR_RG_SX_REFBYTWO*/
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_CBR_SYN_DIV_SDM_XOSC, 
            (0x01<<13), (0x01<<13))) != 0) break;
        
        /* step 2: SET_CBR_RG_SX_RFCTRL_F */
        regval = vt_tbl[chidx].rf_ctrl_F;
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_CBR_SYN_RGISTER_1,
            (regval << 0), 0x00ffffff)) != 0) break;

        /* SET_CBR_RG_SX_RFCTRL_CH */
        regval = vt_tbl[chidx].rf_ctrl_N;
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_SYN_RGISTER_2,
            (regval<<0), 0x000007ff)) != 0) break;

        /* SET_CBR_RG_SX_SUB_SEL_CWD */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_MANUAL_REGISTER,
            (64<<1), (0x000007f<<1))) != 0) break;

        /* SET_CBR_RG_SX_SUB_SEL_CWR */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_MANUAL_REGISTER,
            (1<<0), 0x00000001)) != 0) break;

        /* SET_CBR_RG_SX_SUB_SEL_CWR */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_MANUAL_REGISTER,
            (0<<0), 0x00000001)) != 0) break;
        
        /* step 3: calibration, SET_CBR_RG_EN_SX_VT_MON */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_SX_ENABLE_RGISTER,
            (1<<11), 0x00000800)) != 0) break;
        
        /* SET_CBR_RG_EN_SX_VT_MON_DG */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_SX_ENABLE_RGISTER,
            (0<<12), 0x00001000)) != 0) break;

        /* SET_CBR_RG_EN_SX_VT_MON_DG */
        if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_SX_ENABLE_RGISTER,
            (1<<12), 0x00001000)) != 0) break;

        for(retry_cnt=20; retry_cnt>0; retry_cnt--)
        {
            mdelay(20);

            /* GET_CBR_VT_MON_RDY */
            if ((ret = SMAC_REG_READ(sh, ADR_CBR_READ_ONLY_FLAGS_1, &regval)) != 0) break;
            if (regval & 0x00000004)
            {
                /* SET_CBR_RG_EN_SX_VT_MON_DG */
                if ((ret = SMAC_REG_SET_BITS(sh, ADR_CBR_SX_ENABLE_RGISTER,
                (0<<12), 0x00001000)) != 0) break;

                if ((ret = SMAC_REG_READ(sh, ADR_CBR_READ_ONLY_FLAGS_1, &regval)) != 0) break;
                if ((regval & 0x00001800) == 0)
                {
                    ssv6xxx_rf_enable(sh);
//                    printk("%s(): Lock channel %d success !\n", __FUNCTION__, vt_tbl[chidx].channel_id);

                    return 0;
                }
                // dbg code add by bernie, begin
                else 
                {
                    printk("%s(): Lock channel %d fail!\n", __FUNCTION__, vt_tbl[chidx].channel_id);

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_READ_ONLY_FLAGS_1, &regval)) != 0) break;
                    printk("%s(): dbg: vt-mon read out as %d when rdy\n", __FUNCTION__,  ((regval & 0x00001800) >> 11));

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_READ_ONLY_FLAGS_2, &regval)) != 0) break;
                    printk("%s(): dbg: sub-sel read out as %d when rdy\n", __FUNCTION__, ((regval & 0x00000fe0) >>  5));

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_SYN_DIV_SDM_XOSC, &regval)) != 0) break;
                    printk("%s(): dbg: RG_SX_REFBYTWO read out as %d when rdy\n", __FUNCTION__, ((regval & 0x00002000) >>  13));

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_SYN_RGISTER_1, &regval)) != 0) break;
                    printk("%s(): dbg: RG_SX_RFCTRL_F read out as 0x%08x when rdy\n", __FUNCTION__, ((regval & 0x00ffffff) >>  0));

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_SYN_RGISTER_2, &regval)) != 0) break;
                    printk("%s(): dbg: RG_SX_RFCTRL_CH read out as 0x%08x when rdy\n", __FUNCTION__, ((regval & 0x000007ff) >>  0));

                    if ((ret = SMAC_REG_READ(sh, ADR_CBR_SX_ENABLE_RGISTER, &regval)) != 0) break;
                    printk("%s(): dbg: RG_EN_SX_VT_MON_DG read out as %d when rdy\n", __FUNCTION__, ((regval & 0x00001000) >>  12));
                }
                // dbg code add by bernie, end
            }
        }
        
        fail_cnt++;
        printk("%s(): calibration fail [%d] rounds!!\n", 
                __FUNCTION__, fail_cnt);
        if(fail_cnt == 100)
            return -1;

    } while(ret == 0);

    return ret;
}

#endif



#ifdef CONFIG_SSV_CABRIO_E
static const struct ssv6xxx_calib_table vt_tbl[SSV6XXX_IQK_CFG_XTAL_MAX][14]=
{
    /* Table for Cabrio E: for XOC_26 only */
    {
        {  1, 0xB9, 0x89D89E, 3859},
        {  2, 0xB9, 0xEC4EC5, 3867},
        {  3, 0xBA, 0x4EC4EC, 3875},
        {  4, 0xBA, 0xB13B14, 3883},
        {  5, 0xBB, 0x13B13B, 3891},
        {  6, 0xBB, 0x762762, 3899},
        {  7, 0xBB, 0xD89D8A, 3907},
        {  8, 0xBC, 0x3B13B1, 3915},
        {  9, 0xBC, 0x9D89D9, 3923},
        { 10, 0xBD, 0x000000, 3931},
        { 11, 0xBD, 0x627627, 3939},
        { 12, 0xBD, 0xC4EC4F, 3947},
        { 13, 0xBE, 0x276276, 3955},
        { 14, 0xBF, 0x13B13B, 3974},
    },
    /* Table for Cabrio E: for XOC_40 only */
    {
        {  1, 0xf1, 0x333333, 3859},
        {  2, 0xf1, 0xB33333, 3867},
        {  3, 0xf2, 0x333333, 3875},
        {  4, 0xf2, 0xB33333, 3883},
        {  5, 0xf3, 0x333333, 3891},
        {  6, 0xf3, 0xB33333, 3899},
        {  7, 0xf4, 0x333333, 3907},
        {  8, 0xf4, 0xB33333, 3915},
        {  9, 0xf5, 0x333333, 3923},
        { 10, 0xf5, 0xB33333, 3931},
        { 11, 0xf6, 0x333333, 3939},
        { 12, 0xf6, 0xB33333, 3947},
        { 13, 0xf7, 0x333333, 3955},
        { 14, 0xf8, 0x666666, 3974},
    },
    /* Table for Cabrio E: for XOC_24 only */
    {
        {  1, 0xC9, 0x000000, 3859},
        {  2, 0xC9, 0x6AAAAB, 3867},
        {  3, 0xC9, 0xD55555, 3875},
        {  4, 0xCA, 0x400000, 3883},
        {  5, 0xCA, 0xAAAAAB, 3891},
        {  6, 0xCB, 0x155555, 3899},
        {  7, 0xCB, 0x800000, 3907},
        {  8, 0xCB, 0xEAAAAB, 3915},
        {  9, 0xCC, 0x555555, 3923},
        { 10, 0xCC, 0xC00000, 3931},
        { 11, 0xCD, 0x2AAAAB, 3939},
        { 12, 0xCD, 0x955555, 3947},
        { 13, 0xCE, 0x000000, 3955},
        { 14, 0xCF, 0x000000, 3974},
    }
};

#define FAIL_MAX 100
#define RETRY_MAX 20


int ssv6xxx_set_channel(struct ssv_softc *sc, int ch)
{
    struct ssv_hw *sh=sc->sh;
    int retry_cnt, fail_cnt=0;
    u32 regval;
    int ret = -1;
    int  chidx;
    bool chidx_vld = 0;

    dev_dbg(sc->dev, "Setting channel to %d\n", ch);

#ifdef SSV_IPD
    //Channel 13/14
    if((ch == 13) || (ch == 14))
    {
        if(sc->ipd_channel_touch == 0)
        {
            //0xCE01000C
            SMAC_REG_WRITE(sc->sh, ADR_ABB_REGISTER_1, 0x151559fc);
            //0xCE010008
            SMAC_REG_WRITE(sc->sh, ADR_LDO_REGISTER, 0x00432614);
            ////0xCE010030
            SMAC_REG_WRITE(sc->sh, ADR_RX_ADC_REGISTER, 0x20e80164);
            sc->ipd_channel_touch = 1;
        }
    }
    else
    {
        if(sc->ipd_channel_touch)
        {
            //0xCE01000C
            SMAC_REG_WRITE(sc->sh, ADR_ABB_REGISTER_1, sc->def_setting_CE01000C);
            //0xCE010008
            SMAC_REG_WRITE(sc->sh, ADR_LDO_REGISTER, sc->def_setting_CE010008);
            ////0xCE010030
            SMAC_REG_WRITE(sc->sh, ADR_RX_ADC_REGISTER, sc->def_setting_CE010030);
            sc->ipd_channel_touch = 0;
        }
    }
#endif

    for(chidx = 0; chidx < 14; chidx++) {
        if (vt_tbl[sh->cfg.crystal_type][chidx].channel_id == ch) {
            chidx_vld = 1;
            break;
        }
    }

    if (chidx_vld == 0) {
        dev_dbg(sc->dev, "%s(): fail! channel_id not found in vt_tbl\n", __FUNCTION__);
        goto exit;
    }

    //printk("Turning off SSV WiFi RF.\n");
    if ((ret = ssv6xxx_rf_disable(sc->sh)) != 0) 
        goto exit;

    do {
        /*
            depend on XSOC:  If crystal > 26M XOSC = 1; else XOSC = 0;
        */
        if((sh->cfg.crystal_type == SSV6XXX_IQK_CFG_XTAL_26M) || (sh->cfg.crystal_type == SSV6XXX_IQK_CFG_XTAL_24M))
        {
            if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SYN_DIV_SDM_XOSC,
                (0x00<<13), (0x01<<13))) != 0) break;
        }
        else if(sh->cfg.crystal_type == SSV6XXX_IQK_CFG_XTAL_40M)
        {
            if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SYN_DIV_SDM_XOSC,
                (0x01<<13), (0x01<<13))) != 0) break;
        }
        else
        {
            printk("Illegal xtal setting -- ssv6xxx_set_channel\n");
            BUG_ON(1);
        }

        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SX_LCK_BIN_REGISTERS_I,
            (0x01<<19), (0x01<<19))) != 0) break;

        regval = vt_tbl[sh->cfg.crystal_type][chidx].rf_ctrl_F;
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SYN_REGISTER_1,
            (regval<<0), (0x00ffffff<<0))) != 0) break;
        
        regval = vt_tbl[sh->cfg.crystal_type][chidx].rf_ctrl_N;
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SYN_REGISTER_2,
            (regval<<0), (0x07ff<<0))) != 0) break;

        if ((ret = SMAC_REG_READ(sc->sh, ADR_SX_LCK_BIN_REGISTERS_I, &regval)) != 0) break;
        regval = vt_tbl[sh->cfg.crystal_type][chidx].rf_precision_default;
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_SX_LCK_BIN_REGISTERS_II,
            (regval<<0), (0x1fff<<0))) != 0) break;

        //calibration
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_MANUAL_ENABLE_REGISTER,
            (0x00<<14), (0x01<<14))) != 0) break;
        if ((ret = SMAC_REG_SET_BITS(sc->sh, ADR_MANUAL_ENABLE_REGISTER,
            (0x01<<14), (0x01<<14))) != 0) break;

        retry_cnt = 0;
        do
        {
            mdelay(1);
            if ((ret = SMAC_REG_READ(sc->sh, ADR_READ_ONLY_FLAGS_1, &regval)) != 0) break;
            if (regval & 0x00000002)
            {
                if ((ret = SMAC_REG_READ(sc->sh, ADR_READ_ONLY_FLAGS_2, &regval)) != 0) break;
                /* rf on */
                ret = ssv6xxx_rf_enable(sc->sh);
//Calibration Debug message
#if 0
                printk("Lock to channel %d ([0xce010098]=%x)!!\n", vt_tbl[sh->cfg.crystal_type][chidx].channel_id, regval);
                printk("crystal_type [%d]\n",sh->cfg.crystal_type);

                SMAC_REG_READ(sc->sh, 0xce010040, &regval);
                printk("0xce010040 [%x]\n",regval);
                SMAC_REG_READ(sc->sh, 0xce0100a4, &regval);
                printk("0xce0100a4 [%x]\n",regval);
                SMAC_REG_READ(sc->sh, ADR_DPLL_DIVIDER_REGISTER, &regval);
                printk("0xce010060 [%x]\n",regval);
                SMAC_REG_READ(sc->sh, ADR_SX_ENABLE_REGISTER, &regval);
                printk("0xce010038 [%x]\n",regval);
                SMAC_REG_READ(sc->sh, 0xce01003C, &regval);
                printk("0xce01003C [%x]\n",regval);
                SMAC_REG_READ(sc->sh, ADR_DPLL_FB_DIVIDER_REGISTERS_I, &regval);
                printk("0xce01009c [%x]\n",regval);
                SMAC_REG_READ(sc->sh, ADR_DPLL_FB_DIVIDER_REGISTERS_II, &regval);
                printk("0xce0100a0 [%x]\n",regval);
                printk("[%x][%x][%x]\n",vt_tbl[sh->cfg.crystal_type][chidx].rf_ctrl_N,vt_tbl[sh->cfg.crystal_type][chidx].rf_ctrl_F,vt_tbl[sh->cfg.crystal_type][chidx].rf_precision_default);
#endif
                dev_info(sc->dev, "Lock to channel %d ([0xce010098]=%x)!!\n", vt_tbl[sh->cfg.crystal_type][chidx].channel_id, regval);
                sc->hw_chan = ch;
                goto exit;
            }
            retry_cnt++;  
        }
        while(retry_cnt < RETRY_MAX);

        fail_cnt++;
        printk("calibation fail:[%d]\n", fail_cnt);
    }
    while((fail_cnt < FAIL_MAX) && (ret == 0));

exit:
    //Update TX gain offset from E-FUSE
    //index 1 is the setting for channel 1~channel 7
    if(ch <= 7)
    {
        if(sh->cfg.tx_power_index_1)
        {
            SMAC_REG_READ(sc->sh, ADR_RX_TX_FSM_REGISTER, &regval);
            regval &= RG_TX_GAIN_OFFSET_I_MSK;
            regval |= (sh->cfg.tx_power_index_1 << RG_TX_GAIN_OFFSET_SFT);
            SMAC_REG_WRITE(sc->sh, ADR_RX_TX_FSM_REGISTER, regval);
        }
        else if(sh->cfg.tx_power_index_2)
        {
            SMAC_REG_READ(sc->sh, ADR_RX_TX_FSM_REGISTER, &regval);
            regval &= RG_TX_GAIN_OFFSET_I_MSK;
            SMAC_REG_WRITE(sc->sh, ADR_RX_TX_FSM_REGISTER, regval);
        }
    }
    //index 2 is the setting for channel 8~channel 14
    else
    {
        if(sh->cfg.tx_power_index_2)
        {
            SMAC_REG_READ(sc->sh, ADR_RX_TX_FSM_REGISTER, &regval);
            regval &= RG_TX_GAIN_OFFSET_I_MSK;
            regval |= (sh->cfg.tx_power_index_2 << RG_TX_GAIN_OFFSET_SFT);
            SMAC_REG_WRITE(sc->sh, ADR_RX_TX_FSM_REGISTER, regval);
        }
        else if(sh->cfg.tx_power_index_1)
        {
            SMAC_REG_READ(sc->sh, ADR_RX_TX_FSM_REGISTER, &regval);
            regval &= RG_TX_GAIN_OFFSET_I_MSK;
            SMAC_REG_WRITE(sc->sh, ADR_RX_TX_FSM_REGISTER, regval);
        }
    }
    return ret;

}

#endif









int ssv6xxx_rf_enable(struct ssv_hw *sh)
{
    return SMAC_REG_SET_BITS(sh, 
        //ADR_CBR_HARD_WIRE_PIN_REGISTER,
        0xce010000,
        (0x02<<12), (0x03<<12)
    );
}



int ssv6xxx_rf_disable(struct ssv_hw *sh)
{
    return SMAC_REG_SET_BITS(sh, 
        //ADR_CBR_HARD_WIRE_PIN_REGISTER,
        0xce010000,
        (0x01<<12), (0x03<<12)
    );


}


int ssv6xxx_update_decision_table(struct ssv_softc *sc)
{
    int i;
    for(i=0; i<MAC_DECITBL1_SIZE; i++) {
        SMAC_REG_WRITE(sc->sh, ADR_MRX_FLT_TB0+i*4, 
        sc->mac_deci_tbl[i]);
        SMAC_REG_CONFIRM(sc->sh, ADR_MRX_FLT_TB0+i*4,
        sc->mac_deci_tbl[i]);  
    }
    for(i=0; i<MAC_DECITBL2_SIZE; i++) {
        SMAC_REG_WRITE(sc->sh, ADR_MRX_FLT_EN0+i*4,
        sc->mac_deci_tbl[i+MAC_DECITBL1_SIZE]); 
        SMAC_REG_CONFIRM(sc->sh, ADR_MRX_FLT_EN0+i*4,
        sc->mac_deci_tbl[i+MAC_DECITBL1_SIZE]);   
    }
    return 0;
}



static int ssv6xxx_frame_hdrlen(struct ieee80211_hdr *hdr, bool is_ht)
{
    #define CTRL_FRAME_INDEX(fc) ((hdr->frame_control-IEEE80211_STYPE_BACK_REQ)>>4)
    /* Control Length: BAR, BA, PS-Poll, RTS, CTS, ACK, CF-End, CF-End+CF-Ack */
    u16 fc, CTRL_FLEN[]= { 16, 16, 16, 16, 10, 10, 16, 16 };
    int hdr_len = 24;

    fc = hdr->frame_control;
    if (ieee80211_is_ctl(fc))
        hdr_len = CTRL_FLEN[CTRL_FRAME_INDEX(fc)];
    else if (ieee80211_is_mgmt(fc)) {
        if (ieee80211_has_order(fc))
            hdr_len += ((is_ht==1)? 4: 0);
    }
    else {
        if (ieee80211_has_a4(fc))
            hdr_len += 6;
        if (ieee80211_is_data_qos(fc)) {
            hdr_len += 2;
            if (ieee80211_has_order(hdr->frame_control) &&
                is_ht==true)
                hdr_len += 4;
        }
    }

    
    return hdr_len;
}



#if 0
static void ssv6xxx_dump_tx_desc(struct sk_buff *skb)
{
    struct ssv6200_tx_desc *tx_desc;
    int s;
    u8 *dat;
    
    tx_desc = (struct ssv6200_tx_desc *)skb->data;
    printk(">> Tx Frame:\n");
    for(s=0, dat=skb->data; s<tx_desc->hdr_len; s++) {
        printk("%02x ", dat[sizeof(*tx_desc)+s]);
        if (((s+1)& 0x0F) == 0)
            printk("\n");
    }
    printk("length: %d, c_type=%d, f80211=%d, qos=%d, ht=%d, use_4addr=%d, sec=%d\n", 
        tx_desc->len, tx_desc->c_type, tx_desc->f80211, tx_desc->qos, tx_desc->ht,
        tx_desc->use_4addr, tx_desc->security);
    printk("more_data=%d, sub_type=%x, extra_info=%d\n", tx_desc->more_data,
        tx_desc->stype_b5b4, tx_desc->extra_info);
    printk("fcmd=0x%08x, hdr_offset=%d, frag=%d, unicast=%d, hdr_len=%d\n",
        tx_desc->fCmd, tx_desc->hdr_offset, tx_desc->frag, tx_desc->unicast,
        tx_desc->hdr_len);
    printk("tx_burst=%d, ack_policy=%d, do_rts_cts=%d, reason=%d, payload_offset=%d\n", 
        tx_desc->tx_burst, tx_desc->ack_policy, tx_desc->do_rts_cts, 
        tx_desc->reason, tx_desc->payload_offset);
    printk("fcmdidx=%d, wsid=%d, txq_idx=%d\n",
         tx_desc->fCmdIdx, tx_desc->wsid, tx_desc->txq_idx);
    printk("RTS/CTS Nav=%d, frame_time=%d, crate_idx=%d, drate_idx=%d, dl_len=%d\n",
        tx_desc->rts_cts_nav, tx_desc->frame_consume_time, tx_desc->crate_idx, tx_desc->drate_idx,
        tx_desc->dl_length);

}


static void ssv6xxx_dump_rx_desc(struct sk_buff *skb)
{
    struct ssv6200_rx_desc *rx_desc;

    rx_desc = (struct ssv6200_rx_desc *)skb->data;
    printk(">> RX Descriptor:\n");
    printk("len=%d, c_type=%d, f80211=%d, qos=%d, ht=%d, use_4addr=%d, l3cs_err=%d, l4_cs_err=%d\n",
        rx_desc->len, rx_desc->c_type, rx_desc->f80211, rx_desc->qos, rx_desc->ht, rx_desc->use_4addr,
        rx_desc->l3cs_err, rx_desc->l4cs_err);
    printk("align2=%d, psm=%d, stype_b5b4=%d, extra_info=%d\n", 
        rx_desc->align2, rx_desc->psm, rx_desc->stype_b5b4, rx_desc->extra_info);
    printk("hdr_offset=%d, reason=%d, rx_result=%d\n", rx_desc->hdr_offset,
        rx_desc->reason, rx_desc->RxResult);

}
#endif

/*
 * rix - rate index
 * pktlen - total bytes (delims + data + fcs + pads + pad delims)
 * width  - 0 for 20 MHz, 1 for 40 MHz
 * half_gi - to use 4us v/s 3.6 us for symbol time
 */
static u32 ssv6xxx_ht_txtime(u8 rix, int pktlen, int width, 
                int half_gi, bool is_gf)
{
    u32 nbits, nsymbits, duration, nsymbols;
    int streams;

    /* find number of symbols: PLCP + data */
    streams = 1; /* we only support 1 spatial stream */
    nbits = (pktlen << 3) + OFDM_PLCP_BITS;
    nsymbits = bits_per_symbol[rix % 8][width] * streams;
    nsymbols = (nbits + nsymbits - 1) / nsymbits;

    if (!half_gi)
        duration = SYMBOL_TIME(nsymbols);
    else
    {
        if (!is_gf)
            duration = DIV_ROUND_UP(SYMBOL_TIME_HALFGI(nsymbols), 4)<<2;
        else
        duration = SYMBOL_TIME_HALFGI(nsymbols);
    }

    /* addup duration for legacy/ht training and signal fields */
    duration += L_STF + L_LTF + L_SIG + HT_SIG + HT_STF + HT_LTF(streams)+HT_SIGNAL_EXT;

    if (is_gf)
        duration -=12;

    duration += HT_SIFS_TIME;

    return duration;
}




static u32 ssv6xxx_non_ht_txtime(u8 phy, int kbps,
                                 u32 frameLen, bool shortPreamble)
{
    u32 bits_per_symbol, num_bits, num_symbols; 
    u32 phy_time, tx_time;

    if (kbps == 0)
        return 0;

    switch (phy) {
    case WLAN_RC_PHY_CCK:
        phy_time = CCK_PREAMBLE_BITS + CCK_PLCP_BITS;
        if (shortPreamble)
            phy_time >>= 1;
        num_bits = frameLen << 3;
        tx_time = CCK_SIFS_TIME + phy_time + ((num_bits * 1000) / kbps);
        break;
    case WLAN_RC_PHY_OFDM:
        bits_per_symbol = (kbps * OFDM_SYMBOL_TIME) / 1000;
        num_bits = OFDM_PLCP_BITS + (frameLen << 3);
        num_symbols = DIV_ROUND_UP(num_bits, bits_per_symbol);
        tx_time = OFDM_SIFS_TIME + OFDM_PREAMBLE_TIME
            + (num_symbols * OFDM_SYMBOL_TIME);
        break;
    default:
        printk("Unknown phy %u\n", phy);
                BUG_ON(1);
        tx_time = 0;
        break;
    }

    return tx_time;
}

static u32 ssv6xxx_set_frame_duration(struct ieee80211_tx_info *info,
            struct ssv_rate_info *ssv_rate, u16 len,
            struct ssv6200_tx_desc *tx_desc, struct fw_rc_retry_params *rc_params, 
            struct ssv_softc *sc)
{
    struct ieee80211_tx_rate *tx_drate;
    u32 frame_time=0, ack_time=0, rts_cts_nav=0, frame_consume_time=0;
    u32 l_length=0, drate_kbps=0, crate_kbps=0;
    bool ctrl_short_preamble=false, is_sgi, is_ht40;
    bool is_ht, is_gf;
    int d_phy ,c_phy, nRCParams, mcsidx;
    struct ssv_rate_ctrl *ssv_rc = NULL;

    /**
        * Decide TX rate according to the info from mac80211 protocol.
        * Here we always use the first data rate as the final tx rate.
        * Note the ieee80211_get_tx_rate() always use  info->control.rates[0].
        */      
    tx_drate = &info->control.rates[0];
	is_sgi = !!(tx_drate->flags & IEEE80211_TX_RC_SHORT_GI);
	is_ht40 = !!(tx_drate->flags & IEEE80211_TX_RC_40_MHZ_WIDTH);
    is_ht = !!(tx_drate->flags & IEEE80211_TX_RC_MCS);
    is_gf = !!(tx_drate->flags & IEEE80211_TX_RC_GREEN_FIELD);

    /**
        * We check if Short Premable is needed for RTS/CTS/ACK control
        * frames by checking BSS's global flag. This flags is updated by
        * BSS's beacon frames.
        */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	if ((info->control.short_preamble) || 
		(tx_drate->flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE))
		ctrl_short_preamble = true;
#else
    if ((info->control.vif &&
        info->control.vif->bss_conf.use_short_preamble) || 
		(tx_drate->flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE))
        ctrl_short_preamble = true;
#endif // >= 3.10.0

#ifdef FW_RC_RETRY_DEBUG
    printk("mcs = %d, data rate idx=%d\n",tx_drate->idx, tx_drate[3].count);
#endif
    
    for (nRCParams = 0; (nRCParams < SSV62XX_TX_MAX_RATES) ; nRCParams++)
    {
        if ((rc_params == NULL) || (sc == NULL))
        {
            mcsidx = tx_drate->idx;
            drate_kbps = ssv_rate->drate_kbps;
            crate_kbps = ssv_rate->crate_kbps;
        }
        else
        {
            if(rc_params[nRCParams].count == 0)
            {
                break;
            }
            
            ssv_rc = sc->rc;
            mcsidx = (rc_params[nRCParams].drate - SSV62XX_RATE_MCS_INDEX) % MCS_GROUP_RATES;
            drate_kbps = ssv_rc->rc_table[rc_params[nRCParams].drate].rate_kbps;
            crate_kbps = ssv_rc->rc_table[rc_params[nRCParams].crate].rate_kbps;
        }
        
        /* Calculate data frame transmission time (include SIFS) */
        if (tx_drate->flags & IEEE80211_TX_RC_MCS) {
            frame_time = ssv6xxx_ht_txtime(mcsidx, 
                    len, is_ht40, is_sgi, is_gf);
            d_phy = 0;//no need use this flags in n mode.
        }
        else {
            /**
                    * Calculate frame transmission time for b/g mode:
                    *     frame_time = TX_TIME(frame) + SIFS
                    */
            if ((info->band == IEEE80211_BAND_2GHZ) &&
                        !(ssv_rate->d_flags & IEEE80211_RATE_ERP_G))
                            d_phy = WLAN_RC_PHY_CCK;
                    else
                            d_phy = WLAN_RC_PHY_OFDM;


            frame_time = ssv6xxx_non_ht_txtime(d_phy, drate_kbps, 
                len, ctrl_short_preamble);
        }


        /* get control frame phy 
         *n mode data frmaes also response g mode control frames.
         */

        if ((info->band == IEEE80211_BAND_2GHZ) &&
                        !(ssv_rate->c_flags & IEEE80211_RATE_ERP_G))
                            c_phy = WLAN_RC_PHY_CCK;
                    else
                            c_phy = WLAN_RC_PHY_OFDM;

        /**
            * Calculate NAV duration for data frame. The NAV can be classified
            * into the following cases:
            *    [1] NAV = 0 if the frame addr1 is MC/BC or ack_policy = no_ack
            *    [2] NAV = TX_TIME(ACK) + SIFS if non-A-MPDU frame
            *    [3] NAV = TX_TIME(BA) + SIFS if A-MPDU frame
            */
        if (tx_desc->unicast) {
            ack_time = ssv6xxx_non_ht_txtime(c_phy, 
                crate_kbps, ACK_LEN, ctrl_short_preamble); 



    //        printk("ack_time[%d] d_phy[%d] drate_kbp[%d] c_phy[%d] crate_kbps[%d] \n ctrl_short_preamble[%d] ssv_rate->d_flags[%08x] ssv_rate->c_flags[%08x]\n",
    //           ack_time, d_phy, ssv_rate->drate_kbps, c_phy, ssv_rate->crate_kbps, ctrl_short_preamble, ssv_rate->d_flags, ssv_rate->c_flags);

            /* to do ..... */



        }

        /**
            * Calculate NAV for RTS/CTS-to-Self frame if RTS/CTS-to-Self
            * is needed for the frame transmission:
            *       RTS_NAV = cts_time + frame_time + ack_time
            *       CTS_NAV = frame_time + ack_time
            */
        if (tx_desc->do_rts_cts & IEEE80211_TX_RC_USE_RTS_CTS) {
            rts_cts_nav = frame_time;
            rts_cts_nav += ack_time; 
            rts_cts_nav += ssv6xxx_non_ht_txtime(c_phy, 
                crate_kbps, CTS_LEN, ctrl_short_preamble);

            /**
                    * frame consume time:
                    *     TxTime(RTS) + SIFS + TxTime(CTS) + SIFS + TxTime(DATA)
                    *     + SIFS + TxTime(ACK)
                    */
            frame_consume_time = rts_cts_nav;
            frame_consume_time += ssv6xxx_non_ht_txtime(c_phy, 
                crate_kbps, RTS_LEN, ctrl_short_preamble);
        }else if (tx_desc->do_rts_cts & IEEE80211_TX_RC_USE_CTS_PROTECT) {
            rts_cts_nav = frame_time;
            rts_cts_nav += ack_time;

            /**
                    * frame consume time:
                    *     TxTime(CTS) + SIFS + TxTime(DATA) + SIFS + TxTime(ACK)
                    */
            frame_consume_time = rts_cts_nav;
            frame_consume_time += ssv6xxx_non_ht_txtime(c_phy, 
                crate_kbps, CTS_LEN, ctrl_short_preamble);
        }
        else{;}



        /* Calculate L-Length if using HT mode */
        if (tx_drate->flags & IEEE80211_TX_RC_MCS) {
            /**
                    * Calculate frame transmission time & L-Length if the 
                    * frame is transmitted using HT-MF/HT-GF format: 
                    *
                    *  [1]. ceil[TXTIME-T_SIGEXT-20)/4], plus 3 cause 
                    *         we need to get ceil
                    *  [2]. ceil[TXTIME-T_SIGEXT-20]/4]*3 -3
                    */
            l_length = frame_time - HT_SIFS_TIME;
            l_length = ((l_length-(HT_SIGNAL_EXT+20))+3)>>2;
            l_length += ((l_length<<1) - 3);
        }
        
        if((rc_params == NULL) || (sc == NULL))
        {
            tx_desc->rts_cts_nav = rts_cts_nav;
            tx_desc->frame_consume_time = (frame_consume_time>>5)+1;;
            tx_desc->dl_length = l_length;
            break;
        }
        else
        {
            rc_params[nRCParams].rts_cts_nav = rts_cts_nav;
            rc_params[nRCParams].frame_consume_time = (frame_consume_time>>5)+1;
            rc_params[nRCParams].dl_length = l_length;

            if(nRCParams == 0)
            {
                // Overwrite the indexes of data rate and control rate in TxInfo
                // The values from tx_drate may differ to params from RC
                tx_desc->drate_idx = rc_params[nRCParams].drate;
                tx_desc->crate_idx = rc_params[nRCParams].crate;
                tx_desc->rts_cts_nav = rc_params[nRCParams].rts_cts_nav;
                tx_desc->frame_consume_time = rc_params[nRCParams].frame_consume_time;
                tx_desc->dl_length = rc_params[nRCParams].dl_length;
            }
        }
    }

    return ack_time;
}

static void ssv6200_hw_set_pair_type(struct ssv_hw *sh,u8 type)
{
    u32 temp;
    SMAC_REG_READ(sh,ADR_SCRT_SET,&temp);
    temp = (temp & PAIR_SCRT_I_MSK);
    temp |= (type << PAIR_SCRT_SFT);
    SMAC_REG_WRITE(sh,ADR_SCRT_SET, temp);
}

static void ssv6200_hw_set_group_type(struct ssv_hw *sh,u8 type)
{
    u32 temp;
    SMAC_REG_READ(sh,ADR_SCRT_SET,&temp);
    temp = temp & GRP_SCRT_I_MSK;
    temp |= (type << GRP_SCRT_SFT);
    SMAC_REG_WRITE(sh,ADR_SCRT_SET, temp);

    printk(KERN_ERR "Set group key type %d\n", type);
}


void ssv6xxx_reset_sec_module(struct ssv_softc *sc)
{
    ssv6200_hw_set_group_type(sc->sh, ME_NONE);
    ssv6200_hw_set_pair_type(sc->sh, ME_NONE);
}

#ifdef FW_WSID_WATCH_LIST

static int hw_update_watch_wsid(struct ssv_softc *sc, struct ieee80211_sta *sta, 
        struct ssv_sta_info *sta_info, int sta_idx, int rx_hw_sec, int ops)
{
    int ret = 0;
    int retry_cnt=20;
    struct sk_buff          *skb = NULL;
    struct cfg_host_cmd     *host_cmd;
    struct ssv6xxx_wsid_params *ptr;
        
    // make command packet
    printk("cmd=%d for fw wsid list\n", ops);
    skb = ssv_skb_alloc(HOST_CMD_HDR_LEN + sizeof(struct ssv6xxx_wsid_params));

    if(skb == NULL || sta_info == NULL || sc == NULL)
        return -1;
    
    skb->data_len = HOST_CMD_HDR_LEN + sizeof(struct ssv6xxx_wsid_params);
    skb->len      = skb->data_len;

    host_cmd = (struct cfg_host_cmd *)skb->data;

    host_cmd->c_type = HOST_CMD;
    host_cmd->h_cmd  = (u8)SSV6XXX_HOST_CMD_WSID_OP;
    host_cmd->len    = skb->data_len;

    ptr = (struct ssv6xxx_wsid_params *)host_cmd->dat8;
    ptr->cmd = ops;
    ptr->hw_security = rx_hw_sec;
    
    ptr->wsid_idx = (u8)(sta_idx - SSV_NUM_HW_STA);    
    memcpy(&ptr->target_wsid, &sta->addr[0], 6);    

    while(((sc->sh->hci.hci_ops->hci_send_cmd(skb)) != 0) && (retry_cnt))
    {
        printk(KERN_INFO "WSID cmd=%d retry=%d!!\n", ops, retry_cnt);
        retry_cnt--;
    }
    printk("%s: wsid_idx = %u\n", __FUNCTION__, ptr->wsid_idx);

    ssv_skb_free(skb);
    
    if(ops == SSV6XXXX_WSID_OPS_ADD)
        sta_info->hw_wsid = sta_idx;
            
    return ret;
}
#endif // FW_WSID_WATCH_LIST

static void hw_crypto_key_clear(struct ieee80211_hw *hw, int index, struct ieee80211_key_conf *key,
                                struct ssv_vif_priv_data *vif_priv, struct ssv_sta_priv_data *sta_priv)
{
#ifdef FW_WSID_WATCH_LIST
    struct ssv_softc *sc = hw->priv;

    struct ssv_sta_info *sta_info = NULL;
    
    if ((index == 0) && (sta_priv == NULL))
        return;
#endif // FW_WSID_WATCH_LIST
//    int i;
//    int address = 0x00;
//    u32  sec_key_tbl=sc->sh->hw_sec_key;
//    int wsid = 0;

    if ((index < 0) || (index >= 4))
        return;
    // Not used?
    #if 0
    if(sta_info){
//        wsid = sta_info->hw_wsid;
        sta_info->s_flags &= ~STA_FLAG_ENCRYPT;
    }
    #endif // 0
    /*reset group key index*/
    if (index > 0)
    {
        if (vif_priv)
            vif_priv->group_key_idx = 0;
        if (sta_priv)
            sta_priv->group_key_idx = 0;
    }

#ifdef FW_WSID_WATCH_LIST
    if(sta_priv)
    {
        sta_info = &sc->sta_info[sta_priv->sta_idx];
        if ((index == 0) && (sta_priv->has_hw_decrypt == true) && (sta_info->hw_wsid >= SSV_NUM_HW_STA))
        {        
            hw_update_watch_wsid(sc, sta_info->sta, sta_info, sta_priv->sta_idx, SSV6XXXX_WSID_SEC_PAIRWISE
                , SSV6XXXX_WSID_OPS_DISABLE_CAPS);
        }
    }
    
    if(vif_priv)
    {
        if((index != 0) && !list_empty(&vif_priv->sta_list))
        {            
            struct ssv_sta_priv_data *sta_priv_iter;

            list_for_each_entry(sta_priv_iter, &vif_priv->sta_list, list)
            {
                if (((sta_priv_iter->sta_info->s_flags & STA_FLAG_VALID) == 0) 
                     || (sta_priv_iter->sta_info->hw_wsid < SSV_NUM_HW_STA))
                    continue;
                
                hw_update_watch_wsid(sc, sta_priv_iter->sta_info->sta, 
                    sta_priv_iter->sta_info, sta_priv_iter->sta_idx, SSV6XXXX_WSID_SEC_GROUP
                    , SSV6XXXX_WSID_OPS_DISABLE_CAPS);
            }
        }                   
    }
#endif // FW_WSID_WATCH_LIST

    #if 0
    if (index == 0) {
        /* pairwise key*/
        address = sec_key_tbl+(3*sizeof(struct ssv6xxx_hw_key))
            + wsid*sizeof(struct ssv6xxx_hw_sta_key);

        for(i=0;i<(sizeof(struct ssv6xxx_hw_sta_key)/4);i++)
            SMAC_REG_WRITE(sc->sh, address+i*4, 0x0);
    }
    else{
        //Group key

        address = sec_key_tbl+((index-1)*sizeof(struct ssv6xxx_hw_key));

        for(i=0;i<(sizeof(struct ssv6xxx_hw_key)/4);i++)
            SMAC_REG_WRITE(sc->sh,address+i*4, 0x0);
    }
    #endif // 0
}

static void _set_wep_sw_crypto_key (struct ssv_softc *sc,
                                         struct ssv_vif_info *vif_info,
                                         struct ssv_sta_info *sta_info,
                                         void *param)
{
    struct ssv_sta_priv_data *sta_priv = (struct ssv_sta_priv_data *)sta_info->sta->drv_priv;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif_info->vif->drv_priv;

    sta_priv->has_hw_encrypt = vif_priv->has_hw_encrypt;
    sta_priv->has_hw_decrypt = vif_priv->has_hw_decrypt;
    sta_priv->need_sw_encrypt = vif_priv->need_sw_encrypt;
    sta_priv->need_sw_decrypt = vif_priv->need_sw_decrypt;
#ifdef USE_LOCAL_WEP_CRYPTO    
    sta_priv->crypt = vif_priv->crypt;
    sta_priv->crypt_priv = vif_priv->crypt_priv;
#endif
} // end of - _set_wep_sw_crypto_key -

static void _set_wep_hw_crypto_pair_key (struct ssv_softc *sc,
                                         struct ssv_vif_info *vif_info,
                                         struct ssv_sta_info *sta_info,
                                         void *param)
{
    int                     wsid = sta_info->hw_wsid;
    struct ssv6xxx_hw_sec  *sram_key = (struct ssv6xxx_hw_sec *)param;
    int                     address = 0;
    int                    *pointer = NULL;
#ifdef SSV6200_ECO
    u32                     sec_key_tbl_base = sc->sh->hw_sec_key[0];
#else
    u32                     sec_key_tbl_base = sc->sh->hw_sec_key;
#endif
    u32                     sec_key_tbl = sec_key_tbl_base;
    int                     i;
    u8                     *key = sram_key->sta_key[0].pair.key;
    u32                     key_len = *(u16 *)&sram_key->sta_key[0].reserve[0];

    struct ssv_sta_priv_data *sta_priv = (struct ssv_sta_priv_data *)sta_info->sta->drv_priv;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif_info->vif->drv_priv;

    if (wsid == (-1))
        return;

    sram_key->sta_key[wsid].pair_key_idx  = 0;
    sram_key->sta_key[wsid].group_key_idx = 0;

    sta_priv->has_hw_encrypt = vif_priv->has_hw_encrypt;
    sta_priv->has_hw_decrypt = vif_priv->has_hw_decrypt;
    sta_priv->need_sw_encrypt = vif_priv->need_sw_encrypt;
    sta_priv->need_sw_decrypt = vif_priv->need_sw_decrypt;

    if (wsid != 0)
        memcpy(sram_key->sta_key[wsid].pair.key, key, key_len);

    address =   sec_key_tbl
              + (3*sizeof(struct ssv6xxx_hw_key))
              + wsid*sizeof(struct ssv6xxx_hw_sta_key);
#ifdef SSV6200_ECO
    //Base on 0x80000000 - 0x80070000
    address += (0x10000*wsid);
#endif
    pointer = (int *)&sram_key->sta_key[wsid];
    // Debug code
    #if 0
    printk(KERN_ERR "Set STA %d WEP pairwise key to %08X.", wsid, address);
    printk(KERN_ERR "Set WEP %02X %02X %02X %02X %02X %02X %02X %02X... \n",
           key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]);
    #endif

    /* write to pairwise key and index to right position*/
    for (i = 0; i < (sizeof(struct ssv6xxx_hw_sta_key)/4); i++)
        SMAC_REG_WRITE(sc->sh, address+(i*4), *(pointer++));
} // end of - _set_wep_hw_crypto_pair_key -


static void _set_wep_hw_crypto_group_key (struct ssv_softc *sc,
                                          struct ssv_vif_info *vif_info,
                                          struct ssv_sta_info *sta_info,
                                          void *param)
{
    int                     wsid = sta_info->hw_wsid;
    struct ssv6xxx_hw_sec  *sram_key = (struct ssv6xxx_hw_sec *)param;
    int                     address = 0;
    int                    *pointer = NULL;
    u32                     key_idx = sram_key->sta_key[0].pair_key_idx;    
#ifdef SSV6200_ECO
    u32                     sec_key_tbl_base = sc->sh->hw_sec_key[0];
    u32                     key_len = *(u16 *)&sram_key->sta_key[0].reserve[0];
    u8                     *key = sram_key->group_key[key_idx - 1].key;    
#else
    u32                     sec_key_tbl_base = sc->sh->hw_sec_key;
#endif
    u32                     sec_key_tbl = sec_key_tbl_base;

    struct ssv_sta_priv_data *sta_priv = (struct ssv_sta_priv_data *)sta_info->sta->drv_priv;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif_info->vif->drv_priv;

    if (wsid == (-1))
        return;

    if (wsid != 0)
    {
        sram_key->sta_key[wsid].pair_key_idx = key_idx;
        sram_key->sta_key[wsid].group_key_idx = key_idx;
        sta_priv->has_hw_encrypt = vif_priv->has_hw_encrypt;
        sta_priv->has_hw_decrypt = vif_priv->has_hw_decrypt;
        sta_priv->need_sw_encrypt = vif_priv->need_sw_encrypt;
        sta_priv->need_sw_decrypt = vif_priv->need_sw_decrypt;
    }

#ifdef SSV6200_ECO
    if (wsid != 0)
        memcpy(sram_key->group_key[key_idx - 1].key, key, key_len);

    sec_key_tbl += (0x10000 * wsid);
    /* Write group key */
    address =   sec_key_tbl
              + ((key_idx - 1) * sizeof(struct ssv6xxx_hw_key));
    pointer = (int *)&sram_key->group_key[key_idx - 1];

    {
    int i;
    for (i = 0; i < (sizeof(struct ssv6xxx_hw_key)/4); i++)
        SMAC_REG_WRITE(sc->sh, address+(i*4), *(pointer++));
    }
#endif // SSV6200_ECO
    // printk(KERN_ERR "Set STA %d WEP pairwise key to %08X.", wsid, address);

    /* write group key index to all sta entity*/
    address =   sec_key_tbl
              + (3*sizeof(struct ssv6xxx_hw_key))
              + (wsid*sizeof(struct ssv6xxx_hw_sta_key));
    pointer = (int *)&sram_key->sta_key[wsid];
    /*overwrite first 4 byte of sta_key entity */
    SMAC_REG_WRITE(sc->sh, address, *(pointer));
} // end of - _set_wep_hw_crypto_group_key -


static int hw_crypto_key_write_wep(struct ieee80211_hw *hw,
                                   struct ieee80211_key_conf *key,
                                   u8 algorithm,
                                   struct ssv_vif_info *vif_info)
{
    struct ssv_softc *sc = hw->priv;
    struct ssv6xxx_hw_sec *sramKey = &vif_info->sramKey;
#ifndef SSV6200_ECO
    int address = 0x00;
    //int MAX_STA = SSV_NUM_HW_STA;
    int *pointer=NULL;
    u32  sec_key_tbl=sc->sh->hw_sec_key;
    int i;
#endif
#ifdef FW_WSID_WATCH_LIST
    // MAX_STA = SSV_NUM_STA;
#endif
    // Freddie ToDo: For multiple VIF, how to handle WEP coexist with other security modes?

    if (key->keyidx == 0)
    {
        ssv6xxx_foreach_vif_sta(sc, vif_info, _set_wep_hw_crypto_pair_key, sramKey);
    } 
    else // index != 0
    {
#ifndef SSV6200_ECO
        /* setp 1: write group key*/
        //memcpy(sramKey->group_key[key->keyidx-1].key, key->key, key->keylen);

        address =   sec_key_tbl
                  + ((key->keyidx-1) * sizeof(struct ssv6xxx_hw_key));
        pointer = (int *)&sramKey->group_key[key->keyidx-1];

        for (i=0;i<(sizeof(struct ssv6xxx_hw_key)/4);i++)
           SMAC_REG_WRITE(sc->sh, address+(i*4), *(pointer++));
#endif

         /* setp 2: write key index to each sta*/
        ssv6xxx_foreach_vif_sta(sc, vif_info, _set_wep_hw_crypto_group_key, sramKey);
    }

    return 0; 
}


static void _set_aes_tkip_hw_crypto_group_key (struct ssv_softc *sc,
                                               struct ssv_vif_info *vif_info,
                                               struct ssv_sta_info *sta_info,
                                               void *param)
{
    int                    wsid = sta_info->hw_wsid;
#ifdef SSV6200_ECO
    int                    j;
    u32                    sec_key_tbl_base = sc->sh->hw_sec_key[0];
#else
    u32                    sec_key_tbl_base = sc->sh->hw_sec_key;
#endif
    u32                    sec_key_tbl = sec_key_tbl_base;
    int                    address = 0;
    int                   *pointer = 0;
    struct ssv6xxx_hw_sec *sramKey = &(vif_info->sramKey);
    int                    index = *(int *)param;

    if (wsid == (-1))
        return;

    BUG_ON(index == 0);

    //printk("Set CCMP/TKIP group key %d to WSID %d.\n", index, wsid);
    sramKey->sta_key[wsid].group_key_idx = index;

#ifdef SSV6200_ECO
    sec_key_tbl += (0x10000 * wsid);

    address =   sec_key_tbl
              + ((index-1) * sizeof(struct ssv6xxx_hw_key));

    if (vif_info->vif_priv != NULL)
        dev_info(sc->dev, "Write group key %d to VIF %d to %08X\n",
                 index, vif_info->vif_priv->vif_idx, address);
    else
        dev_err(sc->dev, "NULL VIF.\n");

    pointer = (int *)&sramKey->group_key[index-1];

    /* write group key*/
    for (j = 0; j < (sizeof(struct ssv6xxx_hw_key)/4); j++)
        SMAC_REG_WRITE(sc->sh, address+(j*4), *(pointer++));
#endif // SSV6200_ECO
    //mapping to station
    address =   sec_key_tbl
              + (3*sizeof(struct ssv6xxx_hw_key))
              + (wsid * sizeof(struct ssv6xxx_hw_sta_key));
    pointer = (int *)&sramKey->sta_key[wsid];
    /*overwrite  first 4 byte of sta_key entity */
    SMAC_REG_WRITE(sc->sh, address, *(pointer));
#ifdef FW_WSID_WATCH_LIST
    // Freddie ToDo: WSID mapping separates support hardware security
    // availability for pairwise and group type
        
    if (wsid >= SSV_NUM_HW_STA)
    {
        hw_update_watch_wsid(sc, sta_info->sta, sta_info,
            wsid, SSV6XXXX_WSID_SEC_GROUP, SSV6XXXX_WSID_OPS_ENABLE_CAPS);
    }
#endif

} // end of - _set_aes_tkip_hw_crypto_key -


/* TKIP CCMP*/
static int _write_pairwise_key_to_hw (struct ssv_softc *sc,
                                      int index, u8 algorithm,
                                      const u8 *key, int key_len,
                                      struct ieee80211_key_conf *keyconf,
                                      struct ssv_vif_priv_data *vif_priv,
                                      struct ssv_sta_priv_data *sta_priv)
{
    int                    i;
    struct ssv6xxx_hw_sec *sramKey;
    int                    address = 0;
    int                   *pointer = NULL;
#ifdef SSV6200_ECO
    u32                    sec_key_tbl_base = sc->sh->hw_sec_key[0];
#else
    u32                    sec_key_tbl_base = sc->sh->hw_sec_key;
#endif
    u32                    sec_key_tbl;
    int                    wsid = (-1);

    if (sta_priv == NULL)
    {
        dev_err(sc->dev, "Set pair-wise key with NULL STA.\n");
        return -EOPNOTSUPP;
    }

    wsid = sta_priv->sta_info->hw_wsid;

    if ((wsid < 0) || (wsid >= SSV_NUM_STA))
    {
        dev_err(sc->dev, "Set pair-wise key to invalid WSID %d.\n", wsid);
        return -EOPNOTSUPP;
    }

    #if 0
    sta_info->s_flags |= STA_FLAG_ENCRYPT;
    #endif // 0

    dev_info(sc->dev, "Set STA %d's pair-wise key of %d bytes.\n", wsid, key_len);

    sramKey = &(sc->vif_info[vif_priv->vif_idx].sramKey);

    sramKey->sta_key[wsid].pair_key_idx = 0;
    sramKey->sta_key[wsid].group_key_idx = vif_priv->group_key_idx;
        
    memcpy(sramKey->sta_key[wsid].pair.key, key, key_len);
        
    sec_key_tbl = sec_key_tbl_base;
    #ifdef SSV6200_ECO
    //Base on 0x80000000 - 0x80070000
    sec_key_tbl += (0x10000 * wsid);
    #endif

    address =   sec_key_tbl
              + (3 * sizeof(struct ssv6xxx_hw_key))
              + wsid * sizeof(struct ssv6xxx_hw_sta_key);
    pointer = (int *)&sramKey->sta_key[wsid];

    // Debug code
    // _ssv6xxx_hexdump("Key: ", (const u8 *)pointer, sizeof(struct ssv6xxx_hw_sta_key));
    // dev_err(sc->dev, "Set key to %08X.\n", address);

    /* write pairwise key*/
    for (i = 0; i < (sizeof(struct ssv6xxx_hw_sta_key)/4); i++)
        SMAC_REG_WRITE(sc->sh, (address + (i*4)), *(pointer++));

#ifdef FW_WSID_WATCH_LIST
    // Freddie ToDo: WSID mapping separates support hardware security
    // availability for pairwise and group type
        
    if (wsid >= SSV_NUM_HW_STA)
    {
        hw_update_watch_wsid(sc, sta_priv->sta_info->sta, sta_priv->sta_info,
            sta_priv->sta_idx, SSV6XXXX_WSID_SEC_PAIRWISE, SSV6XXXX_WSID_OPS_ENABLE_CAPS);
    }
#endif
        
    return 0;
} // end of - _write_pairwise_key_to_hw -


static int _write_group_key_to_hw (struct ssv_softc *sc,
                                   int index, u8 algorithm,
                                   const u8 *key, int key_len,
                                   struct ieee80211_key_conf *keyconf,
                                   struct ssv_vif_priv_data *vif_priv,
                                   struct ssv_sta_priv_data *sta_priv)
{
    struct ssv6xxx_hw_sec *sramKey;
#ifndef SSV6200_ECO
    u32                    sec_key_tbl_base = sc->sh->hw_sec_key;
    //u32                    sec_key_tbl;
    int                    address = 0;
    int                   *pointer = NULL;
    int                    i;
#endif
    int                    wsid = sta_priv ? sta_priv->sta_info->hw_wsid : (-1);
    int                    ret = 0;

    if (vif_priv == NULL)
    {
        dev_err(sc->dev, "Setting group key to NULL VIF\n");
        return -EOPNOTSUPP;
    }

    dev_info(sc->dev, "Setting VIF %d group key %d of length %d to WSID %d.\n",
             vif_priv->vif_idx, index, key_len, wsid);

    sramKey = &(sc->vif_info[vif_priv->vif_idx].sramKey);

    /*save group key index */
    vif_priv->group_key_idx = index;

    if (sta_priv)
        sta_priv->group_key_idx = index;

    memcpy(sramKey->group_key[index-1].key, key, key_len);

    #ifndef SSV6200_ECO
    address = sec_key_tbl_base + ((index-1)*sizeof(struct ssv6xxx_hw_key));
    pointer = (int *)&sramKey->group_key[index-1];

    /* write group key*/
    for (i = 0; i < (sizeof(struct ssv6xxx_hw_key)/4); i++)
         SMAC_REG_WRITE(sc->sh, address+(i*4), *(pointer++));
    #endif

    /* write group key index to all sta entity*/
    WARN_ON(sc->vif_info[vif_priv->vif_idx].vif_priv == NULL);
    ssv6xxx_foreach_vif_sta(sc, &sc->vif_info[vif_priv->vif_idx],
                            _set_aes_tkip_hw_crypto_group_key, &index);
    ret = 0;

    return ret;
} // end of - _write_group_key_to_hw -


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static enum SSV_CIPHER_E _prepare_key (struct ieee80211_key_conf *key)
{
    enum SSV_CIPHER_E cipher;

    switch (key->cipher) {
        case WLAN_CIPHER_SUITE_WEP40:
            cipher = SSV_CIPHER_WEP40;
            break;
        case WLAN_CIPHER_SUITE_WEP104:
            cipher = SSV_CIPHER_WEP104;
            break;
        case WLAN_CIPHER_SUITE_TKIP:
            key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
            cipher = SSV_CIPHER_TKIP;
            break;
        case WLAN_CIPHER_SUITE_CCMP:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
            key->flags |= IEEE80211_KEY_FLAG_SW_MGMT;
#else
            key->flags |= (IEEE80211_KEY_FLAG_SW_MGMT_TX | IEEE80211_KEY_FLAG_RX_MGMT);
#endif
            cipher = SSV_CIPHER_CCMP;
            break;
#ifdef CONFIG_SSV_WAPI
        case WLAN_CIPHER_SUITE_SMS4:
            printk("[I] %s, algorithm = WLAN_CIPHER_SUITE_SMS4\n", __func__);
            cipher = SSV_CIPHER_SMS4;
            break;
#endif //CONFIG_SSV_WAPI
        default:
            cipher = SSV_CIPHER_INVALID;
            break;
    }

    return cipher;
} // end of - _prepare_key -
#else
static enum SSV_CIPHER_E _prepare_key (struct ieee80211_key_conf *key)
{
    enum SSV_CIPHER_E cipher;

    switch (key->alg) {
        case ALG_WEP:
            if(key->keylen == 5) // 40 bits / 8 bit_per_byte = 5 bytes
                cipher = SSV_CIPHER_WEP40;
            else
                cipher = SSV_CIPHER_WEP104;
            break;
        case ALG_TKIP:
            cipher = SSV_CIPHER_TKIP;
            key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
            break;
        case ALG_CCMP:
            cipher = SSV_CIPHER_CCMP;
            key->flags |= IEEE80211_KEY_FLAG_SW_MGMT;
            break;
        default:
            cipher = SSV_CIPHER_INVALID;
            break;
    }

    return cipher;
} // end of - _prepare_key -
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)

int _set_key_wep (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                  struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                  struct ieee80211_key_conf *key)
{
    int                            ret = 0;
    struct ssv_vif_info           *vif_info = &sc->vif_info[vif_priv->vif_idx];
    struct ssv6xxx_hw_sec         *sram_key = &vif_info->sramKey;

    // Use STA #0 to store key data.
    sram_key->sta_key[0].pair_key_idx = key->keyidx;
    sram_key->sta_key[0].group_key_idx = key->keyidx;
    *(u16 *)&sram_key->sta_key[0].reserve[0] = key->keylen;

    printk(KERN_ERR "Set WEP %02X %02X %02X %02X %02X %02X %02X %02X... (%d %d)\n",
           key->key[0], key->key[1], key->key[2], key->key[3],
           key->key[4], key->key[5], key->key[6], key->key[7],
           key->keyidx, key->keylen);

    if (key->keyidx == 0)
    {
        memcpy(sram_key->sta_key[0].pair.key, key->key, key->keylen);
    }
    else
    {
        memcpy(sram_key->group_key[key->keyidx - 1].key, key->key, key->keylen);
    }

    #if 1
    if (sc->sh->cfg.use_wpa2_only)
    {
        dev_warn(sc->dev, "Use WPA2 HW security mode only.\n");
    }
    #endif // Debug code
    // WEP uses the same key for pairwise and group frames, i.e. VIF and STA
    // can share crypto.
    // Only first interface can use hardware resource.
    if (   (sc->sh->cfg.use_wpa2_only == 0)
        && vif_priv->vif_idx == 0)
    {
        vif_priv->has_hw_decrypt = true;
        vif_priv->has_hw_encrypt = true;//(vif_priv->force_sw_encrypt == false);
        vif_priv->need_sw_decrypt = false;
        vif_priv->need_sw_encrypt = false;

        ssv6200_hw_set_pair_type(sc->sh, cipher);
        ssv6200_hw_set_group_type(sc->sh, cipher);
        hw_crypto_key_write_wep(sc->hw, key, cipher,
                                &sc->vif_info[vif_priv->vif_idx]);
    }
    else
    #ifdef USE_LOCAL_WEP_CRYPTO
    {
        vif_priv->has_hw_decrypt = false;
        vif_priv->has_hw_encrypt = false;

        // Intialize WEP crypto
        if (vif_priv->crypt && vif_priv->crypt_priv)
        {
            vif_priv->crypt->deinit(vif_priv->crypt_priv);
            vif_priv->crypt = NULL;
            vif_priv->crypt_priv = NULL;
        }

        vif_priv->crypt = get_crypto_wep_ops(); //WEP
        vif_priv->crypt_priv = NULL;

        if (vif_priv->crypt)
            vif_priv->crypt_priv = vif_priv->crypt->init(key->keyidx);

        if (vif_priv->crypt_priv)
        {
            vif_priv->crypt->set_key(key->key, key->keylen, NULL, vif_priv->crypt_priv);

            dev_err(sc->dev, "[Local Crypto]: VIF gets WEP crypto OK!\n");
            dev_err(sc->dev, "[Local Crypto]: Use driver's encrypter.\n");
            vif_priv->need_sw_decrypt = true;
            vif_priv->need_sw_encrypt = true;
        }
        else
        {
            dev_err(sc->dev, "[Local Crypto]: Failed to initialize driver's crypto!\n");
            dev_info(sc->dev, "[Local Crypto]: Use MAC80211's encrypter.\n");
            // Use MAC80211's crypto instead.
            vif_priv->need_sw_decrypt = false;
            vif_priv->need_sw_encrypt = false;
            ret = -EOPNOTSUPP;
        }
        ssv6xxx_foreach_vif_sta(sc, vif_info, _set_wep_sw_crypto_key, NULL);
    }
    #else
    {
        vif_priv->has_hw_decrypt = false;
        vif_priv->has_hw_encrypt = false;
        vif_priv->need_sw_decrypt = false;
        vif_priv->need_sw_encrypt = false;
                
        ssv6xxx_foreach_vif_sta(sc, vif_info, _set_wep_sw_crypto_key, NULL);
        
        ret = -EOPNOTSUPP;
    }
    #endif // USE_LOCAL_WEP_CRYPTO

    vif_priv->pair_cipher = vif_priv->group_cipher = cipher;
    
    vif_priv->is_security_valid = true;

    return ret;
} // end of - _set_key_wep -

static int _set_pairwise_key_tkip_ccmp (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                                        struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                                        struct ieee80211_key_conf *key)
{
    int ret = 0;
    const char *cipher_name = (cipher == SSV_CIPHER_CCMP) ? "CCMP" : "TKIP";

    if (sta_priv == NULL)
    {
        dev_err(sc->dev, "Setting pairwise TKIP/CCMP key to NULL STA.\n");
        return -EOPNOTSUPP;
    }

    #if 1
    if (sc->sh->cfg.use_wpa2_only)
    {
        dev_warn(sc->dev, "Use WPA2 HW security mode only.\n");
    }
    #endif // Debug code

    if (   (vif_priv->vif_idx == 0)
        || ((cipher == SSV_CIPHER_CCMP) && (sc->sh->cfg.use_wpa2_only == 1)))
    {
        sta_priv->has_hw_decrypt = true;
        sta_priv->need_sw_decrypt = false;
        // Hardware encryption is only supported for MPDU. AMPDU connection requires
        // software encryption solution.
        if (   (cipher == SSV_CIPHER_TKIP)
            || (   (   !(sc->sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_TX)
                    || (sta_priv->sta_info->sta->ht_cap.ht_supported == false))
                && (vif_priv->force_sw_encrypt == false)))
        {
            dev_info(sc->dev, "STA %d uses HW encrypter for pairwise.\n", sta_priv->sta_idx);
            sta_priv->has_hw_encrypt = true;
            sta_priv->need_sw_encrypt = false;
            ret = 0;
        }
        else
        {
            sta_priv->has_hw_encrypt = false;
            #ifdef USE_LOCAL_CCMP_CRYPTO
            sta_priv->need_sw_encrypt = true;
            ret = 0;
            #else
            sta_priv->need_sw_encrypt = false;
            ret = -EOPNOTSUPP;
            #endif // USE_LOCAL_CCMP_CRYPTO
        }
    }
    else
    {
        sta_priv->has_hw_encrypt = false;
        sta_priv->has_hw_decrypt = false;
        #ifdef USE_LOCAL_CCMP_CRYPTO
        sta_priv->need_sw_encrypt = true;
        sta_priv->need_sw_decrypt = true;
        ret = 0;
        #else
        dev_err(sc->dev, "STA %d MAC80211's %s cipher.\n", sta_priv->sta_idx, cipher_name);
        sta_priv->need_sw_encrypt = false;
        sta_priv->need_sw_encrypt = false;
        ret = -EOPNOTSUPP;
        #endif // USE_LOCAL_CCMP_CRYPTO
    }

    #ifdef USE_LOCAL_CRYPTO
    if (sta_priv->need_sw_encrypt || sta_priv->need_sw_decrypt)
    {

        if (sta_priv->crypt && sta_priv->crypt_priv)
        {
            sta_priv->crypt->deinit(sta_priv->crypt_priv);
        }

        sta_priv->crypt =   (cipher ==  SSV_CIPHER_CCMP)
                          #ifdef USE_LOCAL_CCMP_CRYPTO
                          ? get_crypto_ccmp_ops()  // CCMP
                          #else
                          ? NULL
                          #endif // USE_LOCAL_CCMP_CRYPTO
                          #ifdef USE_LOCAL_TKIP_CRYPTO
                          : get_crypto_tkip_ops(); // TKIP
                          #else
                          : NULL;
                          #endif // USE_LOCAL_CCMP_CRYPTO
        if (sta_priv->crypt)
            sta_priv->crypt_priv = sta_priv->crypt->init(key->keyidx);

        if (sta_priv->crypt_priv)
        {
            dev_err(sc->dev, "Use driver's %s cipher OK!\n", cipher_name);

            sta_priv->crypt->set_key(key->key, key->keylen, NULL, sta_priv->crypt_priv);
        }
        else
        {
            dev_err(sc->dev, "Failed to initialize driver's %s crypto! "
                    "Use MAC80211's instead.\n", cipher_name);
            // Use MAC80211's crypto instead.
            sta_priv->need_sw_encrypt = false;
            sta_priv->need_sw_decrypt = false;

            vif_priv->need_sw_encrypt = false;
            vif_priv->need_sw_decrypt = false;

            ret = -EOPNOTSUPP;
        }
    }
    #endif // USE_LOCAL_CRYPTO

    if (sta_priv->has_hw_encrypt || sta_priv->has_hw_decrypt)
    {
        ssv6200_hw_set_pair_type(sc->sh, cipher);
#if 0
        ssv6200_hw_set_pair_type(sc->sh, SSV_CIPHER_NONE);
        sta_priv->has_hw_encrypt = false;
        sta_priv->has_hw_decrypt = false;
        sta_priv->need_sw_encrypt = true;
        sta_priv->need_sw_encrypt = true;
#endif
        _write_pairwise_key_to_hw(sc, key->keyidx, cipher,
                                  key->key, key->keylen, key,
                                  vif_priv, sta_priv);

    }

    if (   (vif_priv->has_hw_encrypt || vif_priv->has_hw_decrypt)
        && (vif_priv->group_key_idx > 0))
    {
        _set_aes_tkip_hw_crypto_group_key(sc, &sc->vif_info[vif_priv->vif_idx],
                                          sta_priv->sta_info, &vif_priv->group_key_idx);
    }
    return ret;
} // end of - _set_pairwise_key_tkip_ccmp -

static int _set_group_key_tkip_ccmp (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                                     struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                                     struct ieee80211_key_conf *key)
{
    int ret = 0;
    const char *cipher_name = (cipher == SSV_CIPHER_CCMP) ? "CCMP" : "TKIP";

    vif_priv->group_cipher = cipher;

    #if 1
    if (sc->sh->cfg.use_wpa2_only)
    {
        dev_warn(sc->dev, "Use WPA2 HW security mode only.\n");
    }
    #endif // Debug code

    // Group key encryption is set during pairwise key setting.

    if (   (vif_priv->vif_idx == 0)
        || ((cipher == SSV_CIPHER_CCMP) && (sc->sh->cfg.use_wpa2_only == 1))
       )
    {
        // Hardware encryption is only supported for MPDU. AMPDU connection requires
        // software encryption solution.
        dev_info(sc->dev, "VIF %d uses HW %s cipher for group.\n", vif_priv->vif_idx, cipher_name);
        #ifdef USE_MAC80211_DECRYPT_BROADCAST
        vif_priv->has_hw_decrypt = false;
        ret = -EOPNOTSUPP;
        #else
        vif_priv->has_hw_decrypt = true;
        #endif // USE_MAC80211_DECRYPT_BROADCAST
        vif_priv->has_hw_encrypt = true;
        // Enable software decode in case WSID is not beyond hardware support.
        if ((cipher == SSV_CIPHER_CCMP) && (sc->sh->cfg.use_wpa2_only == 1))
            vif_priv->need_sw_decrypt = true;
        else
            vif_priv->need_sw_decrypt = false;
        vif_priv->need_sw_encrypt = false;
    }
    else
    {
        vif_priv->has_hw_decrypt = false;
        vif_priv->has_hw_encrypt = false;
        #ifdef USE_LOCAL_CRYPTO
        vif_priv->need_sw_encrypt = true;
        vif_priv->need_sw_decrypt = true;
        ret = 0;
        #else
        dev_err(sc->dev, "VIF %d uses MAC80211's %s cipher.\n", vif_priv->vif_idx, cipher_name);
        vif_priv->need_sw_encrypt = false;
        vif_priv->need_sw_encrypt = false;
        ret = -EOPNOTSUPP;
        #endif // USE_LOCAL_CCMP_CRYPTO
    }

    #ifdef USE_LOCAL_CRYPTO
    if (vif_priv->need_sw_encrypt || vif_priv->need_sw_decrypt)
    {
        if (vif_priv->crypt && vif_priv->crypt_priv)
            vif_priv->crypt->deinit(vif_priv->crypt_priv);
        vif_priv->crypt_priv = NULL;

        vif_priv->crypt =   (cipher == SSV_CIPHER_CCMP)
                          #ifdef USE_LOCAL_CCMP_CRYPTO
                          ? get_crypto_ccmp_ops()  // CCMP
                          #else
                          ? NULL
                          #endif // USE_LOCAL_CCMP_CRYPTO
                          #ifdef USE_LOCAL_TKIP_CRYPTO
                          : get_crypto_tkip_ops(); // TKIP
                          #else
                          : NULL;
                          #endif // USE_LOCAL_CCMP_CRYPTO

        if (vif_priv->crypt)
            vif_priv->crypt_priv = vif_priv->crypt->init(key->keyidx);

        if (vif_priv->crypt_priv)
        {
            dev_err(sc->dev, "VIF %d gets %s crypto OK! Use driver's crypto.\n",
                    vif_priv->vif_idx, cipher_name);
            vif_priv->crypt->set_key(key->key, key->keylen, NULL, vif_priv->crypt_priv);
        }
        else
        {
            vif_priv->need_sw_encrypt = false;
            vif_priv->need_sw_decrypt = false;
            dev_err(sc->dev, "VIF %d failed to initialize %s crypto!"
                    " Use MAC80211's instead.\n", vif_priv->vif_idx, cipher_name);
            ret = -EOPNOTSUPP;
        }
    }
    #endif // USE_LOCAL_CRYPTO

    if (vif_priv->has_hw_encrypt || vif_priv->has_hw_decrypt)
    {
        #ifdef USE_MAC80211_DECRYPT_BROADCAST
        ssv6200_hw_set_group_type(sc->sh, ME_NONE);
        #else
        ssv6200_hw_set_group_type(sc->sh, cipher);
        #endif // USE_MAC80211_DECRYPT_BROADCAST

        key->hw_key_idx = key->keyidx;

        _write_group_key_to_hw(sc, key->keyidx, cipher,
                               key->key, key->keylen, key,
                               vif_priv, sta_priv);
    }

    vif_priv->is_security_valid = true;

    return ret;
} // end of - _set_group_key_tkip_ccmp -

static int _set_key_tkip_ccmp (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                               struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                               struct ieee80211_key_conf *key)
{
    if (key->keyidx == 0)
        return _set_pairwise_key_tkip_ccmp(sc, vif_priv, sta_priv, cipher, key);
    else
        return _set_group_key_tkip_ccmp(sc, vif_priv, sta_priv, cipher, key);
} // end of - _set_key_tkip_ccmp -


#ifdef USE_LOCAL_SMS4_CRYPTO

static int _set_pairwise_key_sms4 (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                               struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                               struct ieee80211_key_conf *key)
{
    int ret = 0;

    if (sta_priv == NULL)
    {
        dev_err(sc->dev, "Setting pairwise SMS4 key to NULL STA.\n");
        return -EOPNOTSUPP;
    }

    sta_priv->has_hw_encrypt = false;
    sta_priv->has_hw_decrypt = false;
    sta_priv->need_sw_encrypt = true;
    sta_priv->need_sw_decrypt = true;

    sta_priv->crypt = get_crypto_wpi_ops();

    if (sta_priv->crypt)
        sta_priv->crypt_priv = sta_priv->crypt->init(key->keyidx);

    if (sta_priv->crypt_priv)
    {
        dev_err(sc->dev, "Use driver's SMS4 cipher OK!\n");
        sta_priv->crypt->set_key(key->key, key->keylen, NULL, sta_priv->crypt_priv);
    }
    else
    {
        dev_err(sc->dev, "Failed to initialize driver's SMS4 crypto!\n");
        sta_priv->need_sw_encrypt = false;
        sta_priv->need_sw_decrypt = false;
        ret = -EOPNOTSUPP;
    }

    return ret;
} // end of - _set_pair_key_sms4 -

static int _set_group_key_sms4 (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                                struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                                struct ieee80211_key_conf *key)
{
    int ret = 0;

    vif_priv->has_hw_encrypt = false;
    vif_priv->has_hw_decrypt = false;
    vif_priv->need_sw_encrypt = true;
    vif_priv->need_sw_decrypt = true;

    vif_priv->crypt = get_crypto_wpi_ops();

    if (vif_priv->crypt)
        vif_priv->crypt_priv = vif_priv->crypt->init(key->keyidx);

    if (vif_priv->crypt_priv)
    {
        dev_err(sc->dev, "Use driver's SMS4 cipher OK!\n");
        vif_priv->crypt->set_key(key->key, key->keylen, NULL, vif_priv->crypt_priv);
        vif_priv->is_security_valid = true;
    }
    else
    {
        dev_err(sc->dev, "Failed to initialize driver's SMS4 crypto!\n");
        vif_priv->need_sw_encrypt = false;
        vif_priv->need_sw_decrypt = false;
        ret = -EOPNOTSUPP;
        vif_priv->is_security_valid = false;
    }

    return ret;
} // end of - _set_group_key_sms4 -

static int _set_key_sms4 (struct ssv_softc *sc, struct ssv_vif_priv_data *vif_priv,
                          struct ssv_sta_priv_data *sta_priv, enum SSV_CIPHER_E cipher,
                          struct ieee80211_key_conf *key)
{
	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
        return _set_pairwise_key_sms4(sc, vif_priv, sta_priv, cipher, key);
    else
        return _set_group_key_sms4(sc, vif_priv, sta_priv, cipher, key);
} // end of - _set_key_sms4 -

#endif //USE_LOCAL_SMS4_CRYPTO

static int ssv6200_set_key(struct ieee80211_hw *hw,
                           enum set_key_cmd cmd,
                           struct ieee80211_vif *vif,
                           struct ieee80211_sta *sta,
                           struct ieee80211_key_conf *key)
{
    struct ssv_softc         *sc = hw->priv;
    int                       ret = 0;
    enum SSV_CIPHER_E         cipher = SSV_CIPHER_NONE;
    int                       sta_idx = (-1);
    struct ssv_sta_info      *sta_info = NULL;
    struct ssv_sta_priv_data *sta_priv = NULL;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;

    // Freddie ToDo: Support HW security for 2nd interface.
#if 0
//#ifdef FW_WSID_WATCH_LIST
    int                       another_vif_idx = ((vif_priv->vif_idx + 1) % 2);
    struct ssv_vif_priv_data *another_vif_priv = NULL;
    u32                       another_vif_pair_cipher = 0;
    u32                       another_vif_group_cipher = 0;

    if (sc->vif_info[another_vif_idx].vif)
    {
        another_vif_priv = sc->vif_info[another_vif_idx].vif_priv;
        another_vif_pair_cipher = another_vif_priv->pair_cipher;
        another_vif_group_cipher = another_vif_priv->group_cipher;
    }
#endif // FW_WSID_WATCH_LIST

    if (sta)
    {
        sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
        sta_idx = sta_priv->sta_idx;
        sta_info = sta_priv->sta_info;
    }
    
    BUG_ON((cmd!=SET_KEY) && (cmd!=DISABLE_KEY));
    
    if (!(sc->sh->cfg.hw_caps & SSV6200_HW_CAP_SECURITY))
    {
        dev_warn(sc->dev, "HW does not support security.\n");
        return -EOPNOTSUPP;
    }

    /* hw security just support in valid wsid */
    #ifndef USE_LOCAL_CRYPTO
    if (sta_info && (sta_info->hw_wsid == (-1)))
    {
        dev_warn(sc->dev, "Add STA without HW resource. Use MAC80211's solution.\n");
        return -EOPNOTSUPP;
    }
    #endif

    cipher = _prepare_key(key);

    dev_err(sc->dev, "Set key VIF %d STA %d algorithm = %d, key->keyidx = %d, cmd = %d\n",
            vif_priv->vif_idx, sta_idx, cipher, key->keyidx, cmd);

    if (cipher == SSV_CIPHER_INVALID)
    {
        dev_warn(sc->dev, "Unsupported cipher type.\n");
        return -EOPNOTSUPP;
    }

    mutex_lock(&sc->mutex);

    switch (cmd)
    {
        case SET_KEY:
            {
                // Debug code
                #if 0
                int i;
                printk("================================SET KEY=======================================\n");

                if (sta_info == NULL)
                {
                    printk("NULL STA cmd[%d] alg[%d] keyidx[%d] ", cmd, algorithm, key->keyidx);                
                }
                else
                {
                    printk("STA WSID[%d] cmd[%d] alg[%d] keyidx[%d] ", sta_info->hw_wsid, cmd, algorithm, key->keyidx);                
                }

                printk("SET_KEY index[%d] flags[0x%x] algorithm[%d] key->keylen[%d]\n",
                                                                            key->keyidx, key->flags, algorithm, key->keylen);                
                for(i = 0; i < key->keylen; i++)
                {
                    printk("[%02x]", key->key[i]);
                }
                printk("\n");
                printk("===============================================================================\n");
                #endif // 0
                // dev_info(sc->dev, "Set key %d VIF %p, STA %p\n", key->keyidx, vif, sta);
                // Freddie ToDo: Check on hardware capability to support multiple SSV hardware platform.
                //                And check on VIF capability to support different security type on different VIF.
                switch (cipher)
                {
                    case SSV_CIPHER_WEP40:
                    case SSV_CIPHER_WEP104:
                        ret = _set_key_wep(sc, vif_priv, sta_priv, cipher, key);
                        break;
                    case SSV_CIPHER_TKIP:
                    case SSV_CIPHER_CCMP:
                        ret = _set_key_tkip_ccmp(sc, vif_priv, sta_priv, cipher, key);
                        break;
                    #ifdef CONFIG_SSV_WAPI
                    case SSV_CIPHER_SMS4:
                        ret = _set_key_sms4(sc, vif_priv, sta_priv, cipher, key);
                        break;
                    #endif // CONFIG_SSV_WAPI
                    default:
                        break;
                }
            }
            break;
        case DISABLE_KEY:
            {
#if 0
               printk("================================DEL KEY=======================================\n");
               if(sta_info == NULL){    
                    printk("NULL STA cmd[%d] alg[%d] keyidx[%d] ", cmd, cipher, key->keyidx);                
               }
               else{
                    printk("STA WSID[%d] cmd[%d] alg[%d] keyidx[%d] ", sta_info->hw_wsid, cmd, cipher, key->keyidx);                
               }
                 
                printk("DISABLE_KEY index[%d]\n",key->keyidx);
                printk("==============================================================================\n");
#endif
                #if 0
                if(key->keyidx == 0)
                {
                    sta_info->ampdu_ccmp_encrypt = false;
                }
                #endif // 0
                if ((cipher == ME_TKIP) || (cipher == ME_CCMP))
                {
                    printk(KERN_ERR "Clear key %d VIF %d, STA %d\n",
                           key->keyidx, (vif != NULL), (sta != NULL));
                    hw_crypto_key_clear(hw, key->keyidx, key, vif_priv, sta_priv);
                }
                // if (key->keyidx == 0)
                {
                    if ((key->keyidx == 0) && (sta_priv != NULL))
                    {
                        sta_priv->has_hw_decrypt = false;
                        sta_priv->has_hw_encrypt = false;
                        sta_priv->need_sw_encrypt = false;
                        #ifdef USE_LOCAL_CRYPTO
                        if (sta_priv->crypt && sta_priv->crypt_priv)
                            sta_priv->crypt->deinit(sta_priv->crypt_priv);
                        sta_priv->crypt_priv = NULL;
                        sta_priv->crypt = NULL;
                        #endif // USE_LOCAL_CRYPTO
                    }
                    #ifdef USE_LOCAL_CRYPTO
                    else
                    {
                        if (vif_priv->crypt && vif_priv->crypt_priv)
                            vif_priv->crypt->deinit(vif_priv->crypt_priv);
                        vif_priv->crypt_priv = NULL;
                        vif_priv->crypt = NULL;
                    }
                    #endif // USE_LOCAL_CRYPTO

                    if ((vif_priv->is_security_valid) && (key->keyidx != 0))
                    {
                        #if 0
                        vif_priv->has_hw_decrypt = false;
                        vif_priv->has_hw_encrypt = false;
                        vif_priv->need_sw_encrypt = false;
                        #endif // 0
                        vif_priv->is_security_valid = false;
                    }
                }
                ret = 0;
            }
            break;
        default:
            ret = -EINVAL;
    }

    mutex_unlock(&sc->mutex);
    
    if(sta_priv != NULL)
    {
        printk("sta: hw_en:%d, sw_en:%d, hw_de:%d, sw_de:%d,\n", 
            (sta_priv->has_hw_encrypt==true),(sta_priv->need_sw_encrypt==true),
            (sta_priv->has_hw_decrypt==true),(sta_priv->need_sw_decrypt==true));
    }

    if(vif_priv)
    {
        printk("vif: hw_en:%d, sw_en:%d, hw_de:%d, sw_de:%d, valid:%d\n", 
            (vif_priv->has_hw_encrypt==true),(vif_priv->need_sw_encrypt==true),
            (vif_priv->has_hw_decrypt==true),(vif_priv->need_sw_decrypt==true), (vif_priv->is_security_valid==true));
    }

#ifdef CONFIG_SSV_SW_ENCRYPT_HW_DECRYPT
    ret = -EOPNOTSUPP;
#endif
    
#ifndef USE_LOCAL_CRYPTO
    if (   vif_priv->force_sw_encrypt // AP mode needs SW encryption.
        || (sta_info && (sta_info->hw_wsid != 1) && (sta_info->hw_wsid != 0)))
    {
        // Freddie ToDo: Temporary solution: STA of WSID greater than 1 should be forced to use SW encryption until FW helped to map WSID.
        if (vif_priv->force_sw_encrypt == false)
            vif_priv->force_sw_encrypt = true;
        ret = -EOPNOTSUPP;
    }
#endif // USE_LOCAL_CRYPTO

    printk(KERN_ERR "SET KEY %d\n", ret);
    return ret;
}


u32 _process_tx_done (struct ssv_softc *sc)
{
    struct ieee80211_tx_info   *tx_info;
    struct sk_buff             *skb;

    while ((skb = skb_dequeue(&sc->tx_done_q)))
    {
        struct ssv6200_tx_desc *tx_desc;
        tx_info = IEEE80211_SKB_CB(skb);
        tx_desc = (struct ssv6200_tx_desc *)skb->data;

        if(tx_desc->c_type > M2_TXREQ)
        {
            ssv_skb_free(skb);
            printk(KERN_INFO "free cmd skb!\n");
            continue;
        }

        if (tx_info->flags & IEEE80211_TX_CTL_AMPDU)
        {
            ssv6200_ampdu_release_skb(skb, sc->hw);
            continue;
        }

        skb_pull(skb, SSV6XXX_TX_DESC_LEN);
        ieee80211_tx_info_clear_status(tx_info);
        tx_info->flags |= IEEE80211_TX_STAT_ACK;
        tx_info->status.ack_signal = 100; /* ???? */
#ifdef REPORT_TX_DONE_IN_IRQ
        ieee80211_tx_status_irqsafe(sc->hw, skb);
#else
        ieee80211_tx_status(sc->hw, skb);
        // Break if RX skb is available because TX status report is in RX thread.
        if (skb_queue_len(&sc->rx_skb_q))
            break;
#endif // REPORT_TX_DONE_IN_IRQ
    }

    return skb_queue_len(&sc->tx_done_q);
} // end of - _process_tx_done -


#ifdef REPORT_TX_DONE_IN_IRQ
void ssv6xxx_tx_cb(struct sk_buff_head *skb_head, void *args)
{
    struct ssv_softc *sc=(struct ssv_softc *)args;
    _process_tx_done*(sc);
}
#else // REPORT_TX_DONE_IN_IRQ
void ssv6xxx_tx_cb(struct sk_buff_head *skb_head, void *args)
{
    struct ssv_softc *sc=(struct ssv_softc *)args;
    struct sk_buff *skb;

    while ((skb=skb_dequeue(skb_head)))
    {
        struct ieee80211_tx_info *tx_info = IEEE80211_SKB_CB(skb);
        struct ssv6200_tx_desc *tx_desc;
        tx_desc = (struct ssv6200_tx_desc *)skb->data;

        if(tx_desc->c_type > M2_TXREQ)
        {
            ssv_skb_free(skb);
            printk(KERN_INFO "free cmd skb!\n");
            continue;
        }

        if (tx_info->flags & IEEE80211_TX_CTL_AMPDU)
            ssv6xxx_ampdu_sent(sc->hw, skb);
        skb_queue_tail(&sc->tx_done_q, skb);
    }
    // TX status is reported by RX process thread.
    wake_up_interruptible(&sc->rx_wait_q);
}
#endif // REPORT_TX_DONE_IN_IRQ


// ToDo: Fix RATE_CONTROL_REALTIME_UPDATA typo.
#ifdef RATE_CONTROL_REALTIME_UPDATA
void ssv6xxx_tx_rate_update(struct sk_buff *skb, void *args)
{
    struct ieee80211_hdr *hdr;
    struct ssv_softc *sc = args;
    struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
    struct ssv6200_tx_desc *tx_desc;
    //struct ieee80211_tx_rate *tx_drate;
    struct ssv_rate_info ssv_rate;
    u32 nav=0;
    int ret = 0;

    //hdr = (struct ieee80211_hdr *)(skb->data+(((info->flags & IEEE80211_TX_CTL_AMPDU)? AMPDU_DELIMITER_LEN: 0)+SSV6XXX_TX_DESC_LEN));
    tx_desc = (struct ssv6200_tx_desc *)skb->data;
    if(tx_desc->c_type > M2_TXREQ)
        return;

    if (!(info->flags & IEEE80211_TX_CTL_AMPDU))
    {
        hdr = (struct ieee80211_hdr *)(skb->data+SSV6XXX_TX_DESC_LEN);
        if (   (   ieee80211_is_data_qos(hdr->frame_control)
                || ieee80211_is_data(hdr->frame_control))
            && (tx_desc->wsid < SSV_RC_MAX_HARDWARE_SUPPORT))
        {
            ret = ssv6xxx_rc_hw_rate_update_check(skb, sc, tx_desc->do_rts_cts);

            if (ret & RC_FIRMWARE_REPORT_FLAG)
            {
                {
                    //Free
                    //tx_desc->RSVD_0 = SSV6XXX_RC_COUNTER_CLEAR;
                    //Send result
                    tx_desc->RSVD_0 = SSV6XXX_RC_REPORT;
                    tx_desc->tx_report = 1;
                }
                ret &= 0xf;
            }

            if(ret)
            {
                /* Get rate */
                //tx_drate = &info->control.rates[0];
                ssv6xxx_rc_hw_rate_idx(sc, info, &ssv_rate);

                tx_desc->crate_idx = ssv_rate.crate_hw_idx;
                tx_desc->drate_idx = ssv_rate.drate_hw_idx;
            
                nav = ssv6xxx_set_frame_duration(info, &ssv_rate, skb->len+FCS_LEN, tx_desc, NULL, NULL);
                if (tx_desc->tx_burst == 0)
                {
                    if (tx_desc->ack_policy != 0x01)
                        hdr->duration_id = nav;
                }
            }

        }
    }
    else
    {
    }

    return;
}
#endif // RATE_CONTROL_REALTIME_UPDATA

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#define RTS_CTS_PROTECT(_flg) \
    ((_flg)&IEEE80211_TX_RC_USE_RTS_CTS)? 1: \
    ((_flg)&IEEE80211_TX_RC_USE_CTS_PROTECT)? 2: 0
#endif
//static int dump_count = 0;

void ssv6xxx_update_txinfo (struct ssv_softc *sc, struct sk_buff *skb)
{
    struct ieee80211_hdr            *hdr;
    struct ieee80211_tx_info        *info = IEEE80211_SKB_CB(skb);
//    struct ieee80211_sta          *sta = info->control.sta;
    struct ieee80211_sta            *sta;
    struct ssv_sta_info             *sta_info = NULL;
    struct ssv_sta_priv_data        *ssv_sta_priv = NULL;
    struct ssv_vif_priv_data        *vif_priv = (struct ssv_vif_priv_data *)info->control.vif->drv_priv;
    struct ssv6200_tx_desc          *tx_desc = (struct ssv6200_tx_desc *)skb->data;
    struct ieee80211_tx_rate        *tx_drate;
    struct ssv_rate_info             ssv_rate;
    int                              ac, hw_txqid;
    u32                              nav=0;

    if (info->flags & IEEE80211_TX_CTL_AMPDU)
    {
        struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *)skb->head;
        sta = ampdu_hdr->ampdu_tid->sta;
        hdr = (struct ieee80211_hdr *)(skb->data + TXPB_OFFSET + AMPDU_DELIMITER_LEN);
    }
    else
    {
        struct SKB_info_st *skb_info = (struct SKB_info_st *)skb->head;
        sta = skb_info->sta;
        hdr = (struct ieee80211_hdr *)(skb->data + TXPB_OFFSET);
    }

    /**
     * Note that the 'sta' may be NULL. In case of the NULL condition,
     * we assign WSID to 0x0F always. The NULL condition always
     * happens before associating to an AP.
     */
    if (sta)
    {
        ssv_sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
        sta_info = ssv_sta_priv->sta_info;
    }

    /**
     * Decide frame tid & hardware output queue for outgoing
     * frames. Management frames have a dedicate output queue
     * with higher priority in station mode.
     */
    if ((!sc->bq4_dtim) &&
        (ieee80211_is_mgmt(hdr->frame_control) || 
        ieee80211_is_nullfunc(hdr->frame_control) ||
        ieee80211_is_qos_nullfunc(hdr->frame_control))) {
        ac = 4;
        hw_txqid = 4;
    }
    else if((sc->bq4_dtim) &&
        info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM){
        
        /* In AP mode we use queue 4 to send broadcast frame, 
        when more than one station in sleep mode */
        hw_txqid = 4;
        ac = 4;
    }
    else{
        /* The skb_get_queue_mapping() returns AC */
        ac = skb_get_queue_mapping(skb);
        hw_txqid = sc->tx.hw_txqid[ac]; 
    }   

    /* Get rate */
    tx_drate = &info->control.rates[0];
    ssv6xxx_rc_hw_rate_idx(sc, info, &ssv_rate);

    /**
     * Generate tx info (tx descriptor) in M2 format for outgoing frames.
     * The software MAC of ssv6200 uses M2 format.
     */
    tx_desc->len = skb->len;
    //tx_desc->len = skb->len - sc->sh->tx_desc_len; // Exclude TX descriptor length
    tx_desc->c_type = M2_TXREQ;
    tx_desc->f80211 = 1;
    tx_desc->qos = (ieee80211_is_data_qos(hdr->frame_control))? 1: 0;
    if (tx_drate->flags & IEEE80211_TX_RC_MCS) {
        if (ieee80211_is_mgmt(hdr->frame_control) && 
            ieee80211_has_order(hdr->frame_control))
            tx_desc->ht = 1;
    }
    tx_desc->use_4addr = (ieee80211_has_a4(hdr->frame_control))? 1: 0;
    
    tx_desc->more_data = (ieee80211_has_morefrags(hdr->frame_control))? 1: 0;
    tx_desc->stype_b5b4 = (cpu_to_le16(hdr->frame_control)>>4)&0x3;

    tx_desc->frag = (tx_desc->more_data||(hdr->seq_ctrl&0xf))? 1: 0;    
    tx_desc->unicast = (is_multicast_ether_addr(hdr->addr1)) ? 0: 1;
    tx_desc->tx_burst = (tx_desc->frag)? 1: 0;

    tx_desc->wsid = (!sta_info || (sta_info->hw_wsid < 0)) ? 0x0F : sta_info->hw_wsid;
    tx_desc->txq_idx = hw_txqid;
    tx_desc->hdr_offset = TXPB_OFFSET; //SSV6XXX_TX_DESC_LEN
    tx_desc->hdr_len = ssv6xxx_frame_hdrlen(hdr, tx_desc->ht);
    tx_desc->payload_offset = tx_desc->hdr_offset + tx_desc->hdr_len;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	if(info->control.use_rts)
		tx_desc->do_rts_cts = IEEE80211_TX_RC_USE_RTS_CTS;
	else if(info->control.use_cts_prot)
		tx_desc->do_rts_cts = IEEE80211_TX_RC_USE_CTS_PROTECT;
#else
    tx_desc->do_rts_cts = RTS_CTS_PROTECT(tx_drate->flags);
#endif // >= 3.10.0

    if(tx_desc->do_rts_cts == IEEE80211_TX_RC_USE_CTS_PROTECT)
    {
    	/**
        * Note: if cts-to-self is used, always use B mode rate. Here
        * we use 1Mbps as control rate.
        *
		* All protection frames are transmited at 2Mb/s for 802,11g
		* otherwise we transmit them at 1Mb/s.
		*/
		tx_desc->crate_idx = 0;//1Mbs
    }
    else
        tx_desc->crate_idx = ssv_rate.crate_hw_idx;

    tx_desc->drate_idx = ssv_rate.drate_hw_idx;

    if (tx_desc->unicast == 0)               
        tx_desc->ack_policy = 1; /* no ack */
    else if (tx_desc->qos == 1)
        tx_desc->ack_policy = (*ieee80211_get_qos_ctl(hdr)&0x60)>>5;
    else if(ieee80211_is_ctl(hdr->frame_control))
        tx_desc->ack_policy = 1;/* no ack */

    tx_desc->security = 0;

    tx_desc->fCmdIdx = 0;
    // Packet command flow
    //  - TX queue is always the last one.
    tx_desc->fCmd = (hw_txqid+M_ENG_TX_EDCA0);

    // AMPDU TX frame needs MCU to add RTS protection.
    if (info->flags & IEEE80211_TX_CTL_AMPDU)
    {
        #ifdef AMPDU_HAS_LEADING_FRAME
        tx_desc->fCmd = (tx_desc->fCmd << 4) | M_ENG_CPU;
        #else
        tx_desc->RSVD_1 = 1;
        #endif // AMPDU_HAS_LEADING_FRAME
        tx_desc->aggregation = 1;
        // FW retry AMPDU. Ack policy for MAC TX is no-ack. 
        tx_desc->ack_policy = 0x01;

        // RTS/CTS to protect AMPDU
        if (   (tx_desc->do_rts_cts == 0)
            && (   (sc->hw->wiphy->rts_threshold == (-1)) // Use RTS/CTS for A-MPDU, if threshold is no set.
                || ((skb->len - sc->sh->tx_desc_len) > sc->hw->wiphy->rts_threshold)))
        {
            // Raise RTS/CTS such that NAV would be calculate later by ssv6xxx_set_frame_duration.
            tx_drate->flags |= IEEE80211_TX_RC_USE_RTS_CTS;
            tx_desc->do_rts_cts = 1;
        }
    }

    // Check if need HW encryption
    if (   ieee80211_has_protected(hdr->frame_control)
        && (   ieee80211_is_data_qos(hdr->frame_control)
            || ieee80211_is_data(hdr->frame_control)))
    {
        if (   (tx_desc->unicast && ssv_sta_priv && ssv_sta_priv->has_hw_encrypt)
            || (!tx_desc->unicast && vif_priv && vif_priv->has_hw_encrypt))
        {
            // For multicast frame, find first STA's WSID for group key.
            if (!tx_desc->unicast && !list_empty(&vif_priv->sta_list))
            {
                struct ssv_sta_priv_data *one_sta_priv;
                int                       hw_wsid;
                one_sta_priv = list_first_entry(&vif_priv->sta_list, struct ssv_sta_priv_data, list);
                hw_wsid = one_sta_priv->sta_info->hw_wsid;
                if (hw_wsid != (-1))
                {
                    tx_desc->wsid = hw_wsid;
                }
                //vif_info->
                #if 0
                printk(KERN_ERR "HW ENC %d %02X:%02X:%02X:%02X:%02X:%02X\n",
                       tx_desc->wsid,
                       hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                       hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
                _ssv6xxx_hexdump("M ", (const u8 *)skb->data, (skb->len > 128) ? 128 : skb->len);
                //tx_desc->tx_report = 1;
                tx_desc->fCmd = (tx_desc->fCmd << 4) | M_ENG_CPU;
                #endif

            }
            tx_desc->fCmd = (tx_desc->fCmd << 4) | M_ENG_ENCRYPT;
            // Debug code
            #if 0
            if (dump_count++ < 10)
            {
                printk(KERN_ERR "HW ENC %d %02X:%02X:%02X:%02X:%02X:%02X\n",
                       tx_desc->wsid,
                       hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                       hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
                tx_desc->tx_report = 1;
                _ssv6xxx_hexdump("M ", (const u8 *)skb->data, (skb->len > 128) ? 128 : skb->len);
            }
            #endif
            // Debug code
        }
        else if (ssv_sta_priv->need_sw_encrypt)
        {
            //printk(KERN_ERR "LOCAL ENC\n");
        }
        else
        {
            //printk(KERN_ERR "SW ENC\n");
        }
    }
    else
    {
        //printk(KERN_ERR "NO SEC\n");
    }

    //  - HCI is always at the first position.
    tx_desc->fCmd = (tx_desc->fCmd << 4) | M_ENG_HWHCI;
    // Debug code
    #if 0
    if (   ieee80211_is_data_qos(hdr->frame_control)
        || ieee80211_is_data(hdr->frame_control))
    #endif
    #if 0
    if (ieee80211_is_probe_resp(hdr->frame_control))
    {
        //if (dump_count++ < 30)
        {
            printk(KERN_ERR "Probe Resp %d %02X:%02X:%02X:%02X:%02X:%02X\n",
                   tx_desc->wsid,
                   hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                   hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
            _ssv6xxx_hexdump("M ", (const u8 *)skb->data, (skb->len > 128) ? 128 : skb->len);
        }
    }
    #endif

#if 0
    /* check if need hw security module to encrypt */
    if (   (sc->sh->cfg.hw_caps & SSV6200_HW_CAP_SECURITY)
        && (sc->algorithm != ME_NONE)) {
/*
        //This function is for M0 format.
        if(sta && sta->drv_priv){
            sta_idx = ((struct ssv_sta_priv_data *)sta->drv_priv)->sta_idx;
            if(sc->sta_info[sta_idx].s_flags & STA_FLAG_ENCRYPT) {
                if(ieee80211_is_data(hdr->frame_control))
                    tx_desc->security = 1;
            }
        }
*/
        /* Offload broadcast frame or wep frame to 
                   hw security module and let HW module use
                   WSID 0 to encrypt frame. (AP mode)
               */
        if (   (tx_desc->unicast == 0)
            || (sc->algorithm == ME_WEP104 || sc->algorithm == ME_WEP40))
        {
            tx_desc->wsid = 0;
            //This control bit is for M0 format.
            //tx_desc->security = 1;
        }
    }
#endif

#if 0
    /* Tx-AMPDU patch: 
        * (1) do_rts_cts shall be set to 0 for outgoing AMPDU frame 
        * (2) Sahll not enable hardware security.
        * (3) ack_policy shall be set to 0x01
        */
    if (tx_desc->aggregation) {
        tx_desc->do_rts_cts = 0;
        tx_desc->fCmd = M_ENG_HWHCI|((hw_txqid+M_ENG_TX_EDCA0)<<4); 
        tx_desc->ack_policy = 0x01;
    }
#endif
    /* Calculate all time duration */
    
    if (tx_desc->aggregation == 1)
    {
        struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *)skb->head;
        // Return Hardware rate index (0-38)
        //ssv62xx_ht_rate_update(skb, sc, &tx_desc->rc_params[0]);

        memcpy(&tx_desc->rc_params[0], ampdu_hdr->rates, sizeof(tx_desc->rc_params));
        nav = ssv6xxx_set_frame_duration(info, &ssv_rate, (skb->len+FCS_LEN), tx_desc, &tx_desc->rc_params[0], sc);
        #ifdef FW_RC_RETRY_DEBUG        
        //for (i = 0; i < SSV62XX_TX_MAX_RATES; i++)
        {
            printk("[FW_RC]:param[0]: drate =%d, count =%d, crate=%d, dl_length =%d, frame_consume_time =%d, rts_cts_nav=%d\n", 
                tx_desc->rc_params[0].drate,tx_desc->rc_params[0].count,tx_desc->rc_params[0].crate, 
                tx_desc->rc_params[0].dl_length, tx_desc->rc_params[0].frame_consume_time, tx_desc->rc_params[0].rts_cts_nav);
            printk("[FW_RC]:param[1]: drate =%d, count =%d, crate=%d, dl_length =%d, frame_consume_time =%d, rts_cts_nav=%d\n", 
                tx_desc->rc_params[1].drate,tx_desc->rc_params[1].count,tx_desc->rc_params[1].crate, 
                tx_desc->rc_params[1].dl_length, tx_desc->rc_params[1].frame_consume_time, tx_desc->rc_params[1].rts_cts_nav);
            printk("[FW_RC]:param[2]: drate =%d, count =%d, crate=%d, dl_length =%d, frame_consume_time =%d, rts_cts_nav=%d\n", 
                tx_desc->rc_params[2].drate,tx_desc->rc_params[2].count,tx_desc->rc_params[2].crate, 
                tx_desc->rc_params[2].dl_length, tx_desc->rc_params[2].frame_consume_time, tx_desc->rc_params[2].rts_cts_nav);
        }
        #endif // FW_RC_RETRY_DEBUG
    }
    else
    {
        nav = ssv6xxx_set_frame_duration(info, &ssv_rate, (skb->len+FCS_LEN), tx_desc, NULL, NULL);
    }
    
    /**
        * Assign NAV for outgoing frame. Note that we calculate NAV by driver
        * for HT-GF/HT-MF. The b/g mode NAV is calculated by mac80211
        * stack.
        */
		
	//mac80211 duration calculation is error. need to calculate by driver.		
    if (/*(tx_drate->flags & IEEE80211_TX_RC_MCS) && */(tx_desc->aggregation==0)) {
        if (tx_desc->tx_burst == 0) {
            if (tx_desc->ack_policy != 0x01)
                hdr->duration_id = nav;
//debug
//            printk("duration_id = %d\n", hdr->duration_id);
        }
        else {
            /* tx burst for fragmenetation */
        }
    }
} // end of - ssv6xxx_update_txinfo -


void ssv6xxx_add_txinfo (struct ssv_softc *sc, struct sk_buff *skb)
{
    struct ssv6200_tx_desc          *tx_desc;

    /* Request more spaces in front of the payload for ssv6200 tx info: */
    skb_push(skb, sc->sh->tx_desc_len);
    tx_desc = (struct ssv6200_tx_desc *)skb->data;
    memset((void *)tx_desc, 0, sc->sh->tx_desc_len);
    ssv6xxx_update_txinfo(sc, skb);

} // end of - ssv6xxx_add_txinfo -


int ssv6xxx_get_real_index(struct ssv_softc *sc, struct sk_buff *skb)
{
    struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
    struct ieee80211_tx_rate *tx_drate;
    struct ssv_rate_info ssv_rate;    


    /* Get rate */
    tx_drate = &info->control.rates[0];
    ssv6xxx_rc_hw_rate_idx(sc, info, &ssv_rate);

    return ssv_rate.drate_hw_idx;
}


static void _ssv6xxx_tx (struct ieee80211_hw *hw, struct sk_buff *skb)
{
    struct ssv_softc *sc = hw->priv;
    struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
    struct ieee80211_vif *vif = info->control.vif;        
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
#ifdef USE_LOCAL_CRYPTO
    struct SKB_info_st *skb_info = (struct SKB_info_st *)skb->head;    
    struct ieee80211_sta *sta = skb_info->sta;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)info->control.vif->drv_priv;
    struct ssv_sta_priv_data *ssv_sta_priv =   sta
                                              ? (struct ssv_sta_priv_data *)sta->drv_priv
                                             : NULL;
#endif // USE_LOCAL_CRYPTO
    struct ssv6200_tx_desc *tx_desc;
    int ret;
    unsigned long flags;

    bool send_hci=false;

    do {
        /**
         * Assign sequence number
         */
        if (info->flags & IEEE80211_TX_CTL_ASSIGN_SEQ) {
            if (info->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT)
                sc->tx.seq_no += 0x10;
            hdr->seq_ctrl &= cpu_to_le16(IEEE80211_SCTL_FRAG);
            hdr->seq_ctrl |= cpu_to_le16(sc->tx.seq_no);
        }

        if (sc->dbg_tx_frame) {
            printk("================================================\n");
            _ssv6xxx_hexdump("TX frame", (const u8 *)skb->data, skb->len);
        }

        // Debug code: check AC of EAPOL frame
#if 0
        if (   (skb->protocol == cpu_to_be16(ETH_P_PAE)) // EAPOL frame
            && ieee80211_is_data_qos(hdr->frame_control)) // QoS data
        {
           printk(KERN_ERR "EAPOL frame is %d\n", skb_get_queue_mapping(skb));
        }
#endif 

#ifdef USE_LOCAL_CRYPTO
//#ifndef MULTI_THREAD_ENCRYPT
        if (   
#ifdef MULTI_THREAD_ENCRYPT
            (skb_info->crypt_st == PKT_CRYPT_ST_NOT_SUPPORT) &&
#endif            
            ieee80211_has_protected(hdr->frame_control)
            && (   ieee80211_is_data_qos(hdr->frame_control)
                || ieee80211_is_data(hdr->frame_control)))
        {
            bool unicast = !is_broadcast_ether_addr(hdr->addr1);
            // Freddie ToDo: Handle encryption failure.
            if (   (   unicast
                    && (ssv_sta_priv != NULL) 
                    && ssv_sta_priv->need_sw_encrypt)
                || (   !unicast
                    && vif_priv->is_security_valid 
                    && vif_priv->need_sw_encrypt))
            {
                ssv6xxx_skb_encrypt(skb, sc);
                //printk(KERN_ERR "SW ENC\n");
            }
        }
        else
        {
            //printk(KERN_ERR "NO SEC\n");
        }
//#endif // ifndef MULTI_THREAD_ENCRYPT
#endif // USE_LOCAL_CRYPTO

        /**
         * AMPDU frames: 
         * Each AMPDU frame shall be queued into a dedicate tx queue (
         * per sta & tid) and wait for aggregation.
         */
        // Freddie ToDo: move AMPDU update state to AMPDU.

        if (info->flags & IEEE80211_TX_CTL_AMPDU)
        {
            //Fix channel 14 issue.
            if(ssv6xxx_get_real_index(sc, skb) < SSV62XX_RATE_MCS_INDEX)
            {
                info->flags &= (~IEEE80211_TX_CTL_AMPDU);
                goto tx_mpdu;
            }

            // Freddie ToDo: rekey
            #if 0
            u8 tidno;
            struct ieee80211_sta *sta = info->control.sta;
            struct ssv_sta_priv_data *ssv_sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
            
            tidno = ieee80211_get_qos_ctl(hdr)[0] & IEEE80211_QOS_CTL_TID_MASK;
        
            if((ssv_sta_priv->ampdu_tid[tidno].state == AMPDU_STATE_OPERATION) &&
               (sc->ampdu_rekey_pause < AMPDU_REKEY_PAUSE_ONGOING))
            #endif // 0
            if (ssv6200_ampdu_tx_handler(hw, skb))
            {
                break;
            }
            else
            {
                info->flags &= (~IEEE80211_TX_CTL_AMPDU);
            }
        }
tx_mpdu:

        ssv6xxx_add_txinfo(sc, skb);
        
        /* Put Dtim frame to bcast_queue ???? has vif ???? */
        if( vif && 
            vif->type == NL80211_IFTYPE_AP &&
            (sc->bq4_dtim) &&
    	    info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM )
        {
            struct ssv_vif_priv_data *priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;
            u8 buffered = 0;
            
            spin_lock_irqsave(&sc->ps_state_lock, flags);
            if (priv_vif->sta_asleep_mask)
            {
                /* Queue broadcast instead of sending directly */
                buffered = ssv6200_bcast_enqueue(sc, &sc->bcast_txq, skb);

                if (1 == buffered) {
#ifdef  BCAST_DEBUG                   
                    printk("ssv6200_tx:ssv6200_bcast_start\n");
#endif                    
                    ssv6200_bcast_start(sc);
                }
            }
            spin_unlock_irqrestore(&sc->ps_state_lock, flags);

            /*check if queue in successfully*/
            if (buffered)
                break;
        }

        if (info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM) {
            struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;
            dev_dbg(sc->dev, "vif[%d] sc->bq4_dtim[%d]\n", vif_priv->vif_idx, sc->bq4_dtim);
        }

        /**
            * HCI_SEND() returns the number of buffer left. Returning zero implies
            * there is out of buffer for the next frame. In this case, start the flow 
            * control by calling ieee80211_stop_queue().
            */
        tx_desc = (struct ssv6200_tx_desc *)skb->data;
        ret = HCI_SEND(sc->sh, skb, tx_desc->txq_idx);

        send_hci = true;

    } while (0);

    if(sc->dbg_tx_frame){
        if(send_hci)
            printk("Tx frame send to HCI\n");
        else
            printk("Tx frame queued\n");
        
        printk("================================================\n");
    }
} // end of - _ssv6xxx_tx -

#ifdef MULTI_THREAD_ENCRYPT
// Freddie ToDo: Extract encryption data here to reduce redundant 
//               encryption check.
bool _is_encrypt_needed(struct sk_buff *skb)
{
    struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
    struct SKB_info_st *skb_info = (struct SKB_info_st *)skb->head;
    struct ieee80211_sta *sta = skb_info->sta;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)info->control.vif->drv_priv;
    struct ssv_sta_priv_data *ssv_sta_priv = sta ? (struct ssv_sta_priv_data *)sta->drv_priv : NULL;
    
    if (   (   !ieee80211_is_data_qos(hdr->frame_control)
            && !ieee80211_is_data(hdr->frame_control))
        || !ieee80211_has_protected(hdr->frame_control))
        return false;

    // Broadcast frame
    if (is_broadcast_ether_addr(hdr->addr1))
    {
        if (   vif_priv->is_security_valid
            && vif_priv->need_sw_encrypt
            #ifdef USE_LOCAL_CRYPTO
            && (vif_priv->crypt != NULL)
            #endif // USE_LOCAL_CRYPTO
           )
        {
            return true;
        }
    }
    // Unicast
    else if (ssv_sta_priv != NULL)
    {
        if (   ssv_sta_priv->need_sw_encrypt
            #ifdef USE_LOCAL_CRYPTO
            && (ssv_sta_priv->crypt != NULL)
            #endif // USE_LOCAL_CRYPTO
           )
            return true;
    }

    return false;
}
#endif // MULTI_THREAD_ENCRYPT

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
static int ssv6200_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
static void ssv6200_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
#else 
static void ssv6200_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control, struct sk_buff *skb)
#endif 
{
    struct ssv_softc *sc = (struct ssv_softc *)hw->priv;
    struct SKB_info_st *skb_info = (struct SKB_info_st *)skb->head;
#ifdef MULTI_THREAD_ENCRYPT    
    struct ssv_encrypt_task_list *ta = NULL;
    unsigned long flags;
    int ret = -EOPNOTSUPP;
#endif // MULTI_THREAD_ENCRYPT
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
    skb_info->sta = info->control.sta;
    #else
    skb_info->sta = control ? control->sta : NULL;
    #endif

//#ifndef USE_THREAD_TX
#if 0
    _ssv6xxx_tx(hw, skb);
#else
#ifndef MULTI_THREAD_ENCRYPT
    skb_queue_tail(&sc->tx_skb_q, skb);
    wake_up_interruptible(&sc->tx_wait_q);
#else    
    skb_info->crypt_st = PKT_CRYPT_ST_ENC_DONE;
    if(_is_encrypt_needed(skb))
    {
        skb_info->crypt_st = PKT_CRYPT_ST_ENC_PRE;
        ret = ssv6xxx_skb_pre_encrypt(skb, sc);
    }
    if (ret == 0)
    {
        spin_lock_irqsave(&sc->crypt_st_lock, flags);
        __skb_queue_tail(&sc->preprocess_q, skb);
        spin_unlock_irqrestore(&sc->crypt_st_lock, flags);
        list_for_each_entry(ta, &encrypt_task_head, list)
        {
            if((cpu_online(ta->cpu_no)) && (ta->running == 0))
            {
                wake_up(&ta->encrypt_wait_q);
                break;
            }
        }
    }
    else if(ret == -EOPNOTSUPP)
    {
        skb_info->crypt_st = PKT_CRYPT_ST_NOT_SUPPORT;
        skb_queue_tail(&sc->tx_skb_q, skb);
        wake_up_interruptible(&sc->tx_wait_q);        
    }
    else
    {
        dev_err(sc->dev, "strange fail to pre-encrypt packet/n");
    }
#endif //#ifndef MULTI_THREAD_ENCRYPT
    //printk(KERN_ERR "TX\n");
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)   
    //If kernel version less than 2.6.38. and got error. no need to free sk_buff in this function    
    return NETDEV_TX_OK;
#endif    
}

#ifdef MULTI_THREAD_ENCRYPT
int ssv6xxx_encrypt_task (void *data)
{
    struct ssv_softc *sc = (struct ssv_softc *)data;
    unsigned long flags;
    struct ssv_encrypt_task_list *ta = NULL;

    list_for_each_entry(ta, &encrypt_task_head, list)
    {
        if(ta->encrypt_task == current)
            break;
    }

    while (!kthread_should_stop())
    {        
        struct sk_buff *skb = NULL;
        struct ieee80211_hdr *hdr = NULL;
        unsigned long CPUMask = 0;
        int enc_ret = 0;
        volatile bool wakeup_tx;

        // force task whose cpu is offlined to wait event for rebind CPU at cpu notifier        
        if(skb_queue_len_safe(&sc->preprocess_q) == 0 || (ta->cpu_offline != 0) )
        {
            ta->running = 0;
            set_current_state(TASK_UNINTERRUPTIBLE);
            wait_event_timeout(ta->encrypt_wait_q, ( (ta->cpu_offline == 0) && skb_queue_len_safe(&sc->preprocess_q) ) || kthread_should_stop(), msecs_to_jiffies(60000));
            set_current_state(TASK_RUNNING);
            ta->running = 1;
            CPUMask = *(cpumask_bits(&current->cpus_allowed));
        }
        
        if (kthread_should_stop())
        {
            printk("[MT-ENCRYPT]: Quit Encryption task loop ...\n");
            ta->running = 0;
            break;
        }        

        spin_lock_irqsave(&sc->crypt_st_lock, flags);
        if ((skb = __skb_dequeue(&sc->preprocess_q)) != NULL)
        {                       
            //get SKB_Info, enqueue crypt
            SKB_info * skb_info = (SKB_info *)skb->head;
            __skb_queue_tail(&sc->crypted_q, skb);            
			spin_unlock_irqrestore(&sc->crypt_st_lock, flags);
            
            if(skb_info->crypt_st == PKT_CRYPT_ST_ENC_PRE)
            {
                enc_ret = ssv6xxx_skb_encrypt(skb, sc);

                if (enc_ret == 0)                    
                    skb_info->crypt_st = PKT_CRYPT_ST_ENC_DONE;
                else
                    skb_info->crypt_st = PKT_CRYPT_ST_FAIL;
            }
            else if(skb_info->crypt_st == PKT_CRYPT_ST_DEC_PRE)
            {                                                
                struct ieee80211_sta *sta = skb_info->sta;
                struct ieee80211_rx_status *rxs = IEEE80211_SKB_RXCB(skb);             
                enc_ret = ssv6xxx_skb_decrypt(skb, sta, sc);
                if (enc_ret >= 0)
                {
                    skb_info->crypt_st = PKT_CRYPT_ST_DEC_DONE;
                    hdr = (struct ieee80211_hdr *)(skb->data);
                    hdr->frame_control = hdr->frame_control & ~(cpu_to_le16(IEEE80211_FCTL_PROTECTED));
                    rxs->flag |= (RX_FLAG_DECRYPTED|RX_FLAG_IV_STRIPPED);                      
                }
                else
                {
                    printk("####***decrypt fail, skb = %p\n", skb);
                    skb_info->crypt_st = PKT_CRYPT_ST_FAIL;                    
                }
            }

            if(skb_info->crypt_st == PKT_CRYPT_ST_DEC_DONE)
            {
                spin_lock_irqsave(&sc->crypt_st_lock, flags);
                __skb_unlink(skb, &sc->crypted_q);
                sc->rx.num_pkts ++;
                spin_unlock_irqrestore(&sc->crypt_st_lock, flags);
                local_bh_disable();
                ieee80211_rx(sc->hw, skb);
                local_bh_enable();                                   
            }
            else if (skb_info->crypt_st == PKT_CRYPT_ST_FAIL)
            {
                spin_lock_irqsave(&sc->crypt_st_lock, flags);
                __skb_unlink(skb, &sc->crypted_q);
                spin_unlock_irqrestore(&sc->crypt_st_lock, flags);
                ssv_skb_free(skb);
                skb = NULL;
            }
        }
        else
        {
            spin_unlock_irqrestore(&sc->crypt_st_lock, flags);           
        }

        //printk(KERN_ERR "[MT-ENCRYPT]:Start, cpu:%ld, encrypt:%d, empty:%d\n", 
        //        CPUMask, sc->encrypted_cnt, sc->empty_encrypt_st_cnt);
                
        spin_lock_irqsave(&sc->crypt_st_lock, flags);
        skb = NULL;
        wakeup_tx = false;
        while((skb = skb_peek(&sc->crypted_q)) != NULL)
        {
            SKB_info* skb_info = (SKB_info*)skb->head;
            if(skb_info->crypt_st == PKT_CRYPT_ST_ENC_DONE)
            {
                __skb_unlink(skb, &sc->crypted_q);
                skb_queue_tail(&sc->tx_skb_q, skb);
                wakeup_tx = true;
                skb = NULL;
            }
            else
                break;
        }
        spin_unlock_irqrestore(&sc->crypt_st_lock, flags);
        skb = NULL;
        
        if(wakeup_tx)
        {
            wake_up_interruptible(&sc->tx_wait_q);
        }
    }
    
    return 0;
}
#endif //#ifdef MULTI_THREAD_ENCRYPT

int ssv6xxx_tx_task (void *data)
{
    struct ssv_softc *sc = (struct ssv_softc *)data;
    u32 wait_period = SSV_AMPDU_timer_period / 2;
    printk("SSV6XXX TX Task started.\n");

    while (!kthread_should_stop())
    {
        u32 before_timeout = (-1);

        // ToDo: Wait timeout only if there is partial aggregated AMPDU
		set_current_state(TASK_INTERRUPTIBLE);
        before_timeout = wait_event_interruptible_timeout(sc->tx_wait_q,
                                                          (   skb_queue_len(&sc->tx_skb_q)
                                                           || kthread_should_stop()
                                                           || sc->tx_q_empty),
                                                           msecs_to_jiffies(wait_period));

        if (kthread_should_stop())
        {
            printk("Quit TX task loop...\n");
            // ToDo: free skb in sc->tx_skb_q
            break;
        }
		set_current_state(TASK_RUNNING);

        // Take out TX skb from TX Q and process it.
        do {
            struct sk_buff *tx_skb = skb_dequeue(&sc->tx_skb_q);

            if (tx_skb == NULL)
                break;
            _ssv6xxx_tx(sc->hw, tx_skb);
        } while (1);
        // Wake up by HCI TX queue empty notification in lower layer or timeout.
        if (sc->tx_q_empty || (before_timeout == 0))
        {
            //printk("E %d %d\n", sc->tx_q_empty, before_timeout);
            u32 flused_ampdu = ssv6xxx_ampdu_flush(sc->hw);
            sc->tx_q_empty = false;
            if (flused_ampdu == 0 && before_timeout == 0)
            {
                wait_period *= 2;
                if (wait_period > 1000)
                    wait_period = 1000;
            }
        }
        else
            wait_period = SSV_AMPDU_timer_period / 2;
    }

    return 0;
} // end of - ssv6xxx_tx_task -


int ssv6xxx_rx_task (void *data)
{
    struct ssv_softc *sc = (struct ssv_softc *)data;
    unsigned long     wait_period = msecs_to_jiffies(200); // ms time out to process miss-acked MPDU in BA window.
    unsigned long     last_timeout_check_jiffies = jiffies;
    unsigned long     cur_jiffies;

    printk("SSV6XXX RX Task started.\n");

    while (!kthread_should_stop())
    {
        u32 before_timeout = (-1);

        // To make AMPDU window move when BA or no BA message
        // lost for any reason, wait event for timeout and check long waiting
        // MPDU in BA window.
        set_current_state(TASK_INTERRUPTIBLE);

        before_timeout = wait_event_interruptible_timeout(sc->rx_wait_q,
                                                          (   skb_queue_len(&sc->rx_skb_q)
                                                           || skb_queue_len(&sc->tx_done_q)
                                                           || kthread_should_stop()),
                                                           wait_period);

        if (kthread_should_stop())
        {
            printk("Quit RX task loop...\n");
            // ToDo: free skb in sc->rx_skb_q
            break;
        }
        set_current_state(TASK_RUNNING);

        cur_jiffies = jiffies;

        if (   (before_timeout == 0)
            || time_before((last_timeout_check_jiffies +  wait_period),  cur_jiffies))
        {
            ssv6xxx_ampdu_check_timeout(sc->hw);
            last_timeout_check_jiffies = cur_jiffies;
        }
        // Take out RX skb from RX Q and process it.
        if (skb_queue_len(&sc->rx_skb_q))
            _process_rx_q(sc, &sc->rx_skb_q, NULL);
        // Clear up TX done queue.
        if (skb_queue_len(&sc->tx_done_q))
            _process_tx_done(sc);
    }

    return 0;
} // end of - ssv6xxx_rx_task -


#ifdef CONFIG_SSV_CABRIO_E
struct ssv6xxx_iqk_cfg init_iqk_cfg = {
    // cfg_xtal
    SSV6XXX_IQK_CFG_XTAL_26M,
    // cfg_pa
#ifdef CONFIG_SSV_DPD
    SSV6XXX_IQK_CFG_PA_LI_MPB,
#else
    SSV6XXX_IQK_CFG_PA_DEF,
#endif
    // tssi_trgt
    26,
    // tssi_div
    3,
    // cfg_def_tx_scale_11b
    0x75,
    // cfg_def_tx_scale_11b_p0d5
    0x75,
    // cfg_def_tx_scale_11g
    0x80,
    // cfg_def_tx_scale_11g_p0d5
    0x80,
    // cfg_papd_tx_scale_11b
    0x75,
    // cfg_papd_tx_scale_11b_p0d5
    0x75,
    // cfg_papd_tx_scale_11g
    0xa0,
    // cfg_papd_tx_scale_11g_p0d5
    0xa0,
    // cmd_sel
    SSV6XXX_IQK_CMD_INIT_CALI,
    // fx_sel
    { SSV6XXX_IQK_TEMPERATURE
    + SSV6XXX_IQK_RXDC
    + SSV6XXX_IQK_RXRC
    + SSV6XXX_IQK_TXDC
    + SSV6XXX_IQK_TXIQ
    + SSV6XXX_IQK_RXIQ
#ifdef CONFIG_SSV_DPD
    + SSV6XXX_IQK_PAPD
#endif
    },
};
#endif // CONFIG_SSV_CABRIO_E


static int ssv6200_start(struct ieee80211_hw *hw)
{
    struct ssv_softc *sc=hw->priv;
    struct ssv_hw *sh=sc->sh;
    struct ieee80211_channel *chan;

    mutex_lock(&sc->mutex);
    if(sc->ps_status == PWRSV_ENABLE){
        HCI_WAKEUP_PMU(sc->sh);
        msleep(200);
    }
    /* Reset MAC & Re-Init */
    /* Initialize ssv6200 mac */
    if (ssv6xxx_init_mac(sc->sh) != 0) {
        printk("Initialize ssv6200 mac fail!!\n");
        ssv6xxx_deinit_mac(sc);
        return -1;
    }

#ifdef CONFIG_P2P_NOA
    /*Reset NOA*/
    ssv6xxx_noa_reset(sc);
#endif    

#ifdef CONFIG_SSV_CABRIO_E
    /* Do RF-IQ cali. */
    ssv6xxx_do_iq_calib(sc->sh, &init_iqk_cfg);
#endif // CONFIG_SSV_CABRIO_E


#ifdef SSV_IPD
    //For restore channel 1-12 default value
    //0xCE01000C
    SMAC_REG_READ(sc->sh, ADR_ABB_REGISTER_1, &sc->def_setting_CE01000C);
    //0xCE010008
    SMAC_REG_READ(sc->sh, ADR_LDO_REGISTER, &sc->def_setting_CE010008);
    //0xCE010030
    SMAC_REG_READ(sc->sh, ADR_RX_ADC_REGISTER, &sc->def_setting_CE010030);
#endif

    /* get the current channel */
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    chan = hw->conf.channel;
    #else
    chan = hw->conf.chandef.chan;
    #endif
    sc->cur_channel = chan;
    printk("%s(): current channel: %d,sc->ps_status=%d\n", __FUNCTION__, sc->cur_channel->hw_value,sc->ps_status);
    ssv6xxx_set_channel(sc, chan->hw_value);

    /* reset hardware to apply the configuration from mac80211 */


    /* setup interrupt mask of hardware interface (SDIO/SPI) */

    ieee80211_wake_queues(hw);

    ssv6200_ampdu_init(hw);

    HCI_START(sh);
    ssv6xxx_rf_enable(sh);
    mutex_unlock(&sc->mutex);
    return 0;
}


static void ssv6200_stop(struct ieee80211_hw *hw)
{
    struct ssv_softc *sc=hw->priv;
    u32 count=0;

    printk(KERN_INFO "%s(): sc->ps_status=%d\n", __FUNCTION__,sc->ps_status);
    mutex_lock(&sc->mutex);

    ssv6200_ampdu_deinit(hw);

    ssv6xxx_rf_disable(sc->sh);
    
    HCI_STOP(sc->sh);

    /* make sure there is no frame in rx queue */
#ifndef NO_USE_RXQ_LOCK
    while(0) {//(sc->rx.rxq_count > 0)  {
#else
    while (skb_queue_len(&sc->rx.rxq_head)) {
#endif
        printk("sc->rx.rxq_count=%d\n", sc->rx.rxq_count);
        count ++;
        if (count > 90000000) {
            printk("ERROR....ERROR......ERROR..........\n");
            break;
        }
    }
    /* flush tx queue */
    HCI_TXQ_FLUSH(sc->sh, (TXQ_EDCA_0|TXQ_EDCA_1|TXQ_EDCA_2|
        TXQ_EDCA_3|TXQ_MGMT));
    
// Fredie ToDo: Remove RX worker? Notify RX thread to flush out queued packets?
#if 0
//#ifndef DCONFIG_SSV_RX_NO_WORKER
    cancel_work_sync(&sc->rx_work);
#endif

    if((sc->ps_status == PWRSV_PREPARE)||(sc->ps_status == PWRSV_ENABLE)){
           ssv6xxx_enable_ps(sc);
           ssv6xxx_rf_enable(sc->sh);
    }

    mutex_unlock(&sc->mutex);
    
    printk("%s(): leave\n", __FUNCTION__);
}

void inline ssv62xxx_set_bssid(struct ssv_softc *sc, u8 *bssid)
{
    /* Set BSSID to hardware and enable WSID entry 0 */        
    memcpy(sc->bssid, bssid, 6);
    SMAC_REG_WRITE(sc->sh, ADR_BSSID_0, *((u32 *)&sc->bssid[0]));
    SMAC_REG_WRITE(sc->sh, ADR_BSSID_1, *((u32 *)&sc->bssid[4]));
    ssv6xxx_reset_sec_module(sc);
}


struct ssv_vif_priv_data * ssv6xxx_config_vif_res(struct ssv_softc *sc, 
    struct ieee80211_vif *vif)
{
    int                       i;
    struct ssv_vif_priv_data *priv_vif;
    struct ssv_vif_info      *vif_info;

    lockdep_assert_held(&sc->mutex);
    
    //find empty entity
    for(i=0 ; i<SSV6200_MAX_VIF ;i++){
        if (sc->vif_info[i].vif == NULL)
            break;
    }

    BUG_ON(i >= SSV6200_MAX_VIF);
    
    printk("ssv6xxx_config_vif_res id[%d].\n", i);
            
    priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;
    memset(priv_vif, 0, sizeof(struct ssv_vif_priv_data));

    priv_vif->vif_idx = i;

    memset(&sc->vif_info[i], 0, sizeof(sc->vif_info[0]));

    sc->vif_info[i].vif = vif;
    sc->vif_info[i].vif_priv = priv_vif;

    INIT_LIST_HEAD(&priv_vif->sta_list);

    priv_vif->pair_cipher = 0;  // Security type for unicast data of this VIF.
    priv_vif->group_cipher = 0; // Security type for multicast data of this VIF.

    priv_vif->has_hw_decrypt = false;
    priv_vif->has_hw_encrypt = false;
    priv_vif->need_sw_encrypt = false;
    priv_vif->need_sw_decrypt = false;
    priv_vif->is_security_valid = false;
    priv_vif->force_sw_encrypt = (vif->type == NL80211_IFTYPE_AP);

    #ifdef USE_LOCAL_CRYPTO
    priv_vif->crypt = NULL;
    priv_vif->crypt_priv = NULL;
    #endif // USE_LOCAL_CRYPTO

    vif_info = &sc->vif_info[priv_vif->vif_idx];
    vif_info->if_type = vif->type;
    vif_info->vif = vif;
    
    return priv_vif;
}


static int ssv6200_add_interface(struct ieee80211_hw *hw,
                                 struct ieee80211_vif *vif)
{
    struct ssv_softc *sc=hw->priv;
    int ret=0;
//    u32 op_mode = SSV6200_OPMODE_STA;
    struct ssv_vif_priv_data *vif_priv = NULL;

    printk("[I] %s(): \n", __FUNCTION__);

    // AP mode can only exist with managed(station) mode.
    if (   (sc->nvif >= SSV6200_MAX_VIF)
        || (   (   (vif->type == NL80211_IFTYPE_AP)
                || (vif->p2p))
            && (sc->ap_vif != NULL)))
    {
        dev_err(sc->dev, "Add interface of type %d (p2p: %d) failed.\n", vif->type, vif->p2p);
        return -EOPNOTSUPP;
    }

    mutex_lock(&sc->mutex);

    vif_priv = ssv6xxx_config_vif_res(sc, vif);

//temporary disable

    /*it is first entity, and in ap mode*/
    if ((vif_priv->vif_idx == 0) && (vif->type == NL80211_IFTYPE_AP))
    {
        printk("VIF[0] set bssid and config opmode to ap\n");
        ssv62xxx_set_bssid(sc, sc->sh->cfg.maddr[0]);
        SMAC_REG_SET_BITS(sc->sh, ADR_GLBLE_SET, SSV6200_OPMODE_AP, OP_MODE_MSK);
    }

   /* Setup q4 behavior AP mode->xmit frame after DTIM,
    * when we add a normal AP mode vif.
    **/
    if (vif->type == NL80211_IFTYPE_AP)
    {
        BUG_ON(sc->ap_vif != NULL);
        sc->ap_vif = vif;

        // Pure AP mode only?
        if (   !vif->p2p
            && (vif_priv->vif_idx == 0))
        {
            printk("Normal AP mode. Config Q4 to DTIM Q.\n");
            
            SMAC_REG_SET_BITS(sc->sh, ADR_MTX_BCN_EN_MISC,
                              MTX_HALT_MNG_UNTIL_DTIM_MSK, 
                              MTX_HALT_MNG_UNTIL_DTIM_MSK);
            sc->bq4_dtim = true;
        }

#ifdef CONFIG_SSV_SUPPORT_ANDROID
        printk(KERN_INFO "AP mode init wifi_alive_lock\n");
        wake_lock_init(&sc->wifi_alive_lock, WAKE_LOCK_SUSPEND, "wifi alive");
        wake_lock(&sc->wifi_alive_lock);
#endif    
    }
        
    sc->nvif++;
#if 0
    switch(vif->type) {
    case NL80211_IFTYPE_STATION:
        sc->mac_deci_tbl = sta_deci_tbl;
        //op_mode = SSV6200_OPMODE_STA;    default value.
        break;
    case NL80211_IFTYPE_AP:
        sc->mac_deci_tbl = ap_deci_tbl;
        op_mode = SSV6200_OPMODE_AP;
        ssv62xxx_set_bssid(sc, vif, sc->sh->cfg.maddr);
#ifdef CONFIG_SSV_SUPPORT_ANDROID
        printk(KERN_INFO "AP mode init wifi_alive_lock\n");
        wake_lock_init(&sc->wifi_alive_lock, WAKE_LOCK_SUSPEND, "wifi alive");
        wake_lock(&sc->wifi_alive_lock);
#endif    
        break;
    default:
        dev_err(sc->dev, "Unsupported interface type %d is requested.\n", vif->type);
        ret = -EOPNOTSUPP;
        goto out;
    }
    
    sc->vif = vif;
    sc->if_type = vif->type;
    // printk("%s(): Attach a VIF of type: %d\n", __FUNCTION__, vif->type);

    /**
     * Set ssv6200 mac decision table for hardware. The table
     * selection is according to the type of wireless interface:
     * AP & STA mode.
     */
    ssv6xxx_update_decision_table(sc);
    /* update opmode */
    SMAC_REG_SET_BITS(sc->sh, ADR_GLBLE_SET, op_mode, OP_MODE_MSK);

    /* Setup q4 behavior AP mode->xmit frame after DTIM, 
      * STA mode->just like normal queue 
      */
    SMAC_REG_SET_BITS(sc->sh, ADR_MTX_BCN_EN_MISC, 
                (op_mode==SSV6200_OPMODE_STA?0:1),MTX_HALT_MNG_UNTIL_DTIM_MSK);
#endif // 0

    dev_err(sc->dev, "VIF %02x:%02x:%02x:%02x:%02x:%02x of type %d is added.\n",
             vif->addr[0], vif->addr[1], vif->addr[2],
             vif->addr[3], vif->addr[4], vif->addr[5], vif->type);

#ifdef CONFIG_SSV6XXX_DEBUGFS
    ssv6xxx_debugfs_add_interface(sc, vif);
#endif

//out:
    mutex_unlock(&sc->mutex);
    return ret;
}


static void ssv6200_remove_interface(struct ieee80211_hw *hw,
    struct ieee80211_vif *vif)
{
    struct ssv_softc *sc=hw->priv;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;

    dev_err(sc->dev,
             "Removing interface %02x:%02x:%02x:%02x:%02x:%02x. PS=%d\n",
             vif->addr[0], vif->addr[1], vif->addr[2],
             vif->addr[3], vif->addr[4], vif->addr[5], sc->ps_status);
    /** 
        * STA mode decision table is the default table for ssv6xxx.
        * Set the table to default when interface is removed.
        */
    mutex_lock(&sc->mutex);

    #ifdef USE_LOCAL_CRYPTO
    if (vif_priv->crypt && vif_priv->crypt_priv)
    {
        vif_priv->crypt->deinit(vif_priv->crypt_priv);
    }
    vif_priv->crypt = NULL;
    vif_priv->crypt_priv = NULL;
    #endif // USE_LOCAL_CRYPTO

    #ifdef CONFIG_SSV6XXX_DEBUGFS
    ssv6xxx_debugfs_remove_interface(sc, vif);
    #endif

    #if 0
    if ((sc->ps_status == PWRSV_PREPARE)||(sc->ps_status == PWRSV_ENABLE))
    {
           //ssv6xxx_enable_ps(sc);
    } 
    else if (sc->ps_status == PWRSV_DISABLE)
    {
            sc->mac_deci_tbl = sta_deci_tbl;
            ssv6xxx_update_decision_table(sc);
    }
    #endif // 0
    //printk(KERN_INFO "[I] %s(): ,sc->ps_status=%d\n", __FUNCTION__,sc->ps_status);
        
    if (vif->type == NL80211_IFTYPE_AP) 
    {
        /* In normal ap mode, release bcast frame and stop worker */
        if (sc->bq4_dtim)
        {
            sc->bq4_dtim = false;
            
            ssv6200_release_bcast_frame_res(sc, vif);
            SMAC_REG_SET_BITS(sc->sh, ADR_MTX_BCN_EN_MISC,
                              0, MTX_HALT_MNG_UNTIL_DTIM_MSK);
            
            printk("Config Q4 to normal Q \n");
        }

        /* Relase skb of beacon frame */
        ssv6xxx_beacon_release(sc);
        sc->ap_vif = NULL;

#ifdef CONFIG_SSV_SUPPORT_ANDROID
        wake_unlock(&sc->wifi_alive_lock);        
        wake_lock_destroy(&sc->wifi_alive_lock);
        printk(KERN_INFO "AP mode destroy wifi_alive_lock\n");
#endif
    }

    memset(&sc->vif_info[vif_priv->vif_idx], 0, sizeof(struct ssv_vif_info));
    //sc->vif[vif_priv->vif_idx] = NULL;
    sc->nvif--;
    
    mutex_unlock(&sc->mutex);
}


void ssv6xxx_ps_callback_func(unsigned long data)
{
    struct ssv_softc *sc = (struct ssv_softc *)data;
    struct sk_buff *skb;
    struct cfg_host_cmd *host_cmd;
    int retry_cnt=20;

    SMAC_REG_WRITE(sc->sh, ADR_RX_FLOW_MNG,  M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8));
    SMAC_REG_WRITE(sc->sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8));
    SMAC_REG_WRITE(sc->sh, ADR_MRX_FLT_TB0+6*4, (sc->mac_deci_tbl[6]|1));   //ACTION_DO_NOTHING, FRAME_DROP 0x11F8  /* 0 001000 111111 00 1 */       //Beacon

    skb = ssv_skb_alloc(sizeof(struct cfg_host_cmd));
    skb->data_len = sizeof(struct cfg_host_cmd);
    skb->len = skb->data_len;
    host_cmd = (struct cfg_host_cmd *)skb->data;
    host_cmd->c_type = HOST_CMD;
    host_cmd->RSVD0 = 0;//(eCmdID>>8)&0x1F;
    host_cmd->h_cmd = (u8)SSV6XXX_HOST_CMD_PS;
    host_cmd->len = skb->data_len;
    //host_cmd->cmd_seq_no = SSV6XXX_HOST_SOC_CMD_MAXID;
    host_cmd->dummy = sc->ps_aid;
    sc->ps_aid = 0;
    
    while((HCI_SEND_CMD(sc->sh, skb)!=0)&&(retry_cnt)){
        printk(KERN_INFO "PS cmd retry=%d!!\n",retry_cnt);
        retry_cnt--;
    }
    ssv_skb_free(skb);
    //HCI_SEND(sc->sh, skb, 3);
    sc->ps_status = PWRSV_ENABLE;
    printk(KERN_INFO "SSV6XXX_HOST_CMD_PS,ps_aid = %d,len=%d,tabl=0x%x\n",host_cmd->dummy,skb->len,(sc->mac_deci_tbl[6]|1));
}

void ssv6xxx_enable_ps(struct ssv_softc *sc)
{
    sc->ps_status = PWRSV_ENABLE;
    //queue_work(sc->config_wq,&sc->set_ps_work);
    ssv6xxx_ps_callback_func((unsigned long)sc);
    //printk(KERN_INFO "PowerSave enabled\n");
}

void ssv6xxx_disable_ps(struct ssv_softc *sc)
{    
    if(sc->ps_status == PWRSV_ENABLE){
        //SMAC_REG_WRITE(sc->sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_ENCRYPT_SEC<<4)|(M_ENG_HWHCI<<8));
        //SMAC_REG_WRITE(sc->sh, ADR_RX_FLOW_MNG,  M_ENG_MACRX|(M_ENG_HWHCI<<4));
        //SMAC_REG_WRITE(sc->sh, ADR_MRX_FLT_TB0+6*4, (sc->mac_deci_tbl[6]));   //ACTION_DO_NOTHING, FRAME_ACCEPT 0x11F8  /* 0 001000 111111 00 1 */       //Beacon

        //SMAC_LOAD_FW(sc->sh);
        //HCI_WAKEUP_PMU(sc->sh);
        printk(KERN_INFO "PowerSave reg restore\n");
    }
    sc->ps_status = PWRSV_DISABLE;        
    printk(KERN_INFO "PowerSave disabled\n");
}

#ifdef CONFIG_SSV_SUPPORT_ANDROID
#ifdef CONFIG_HAS_EARLYSUSPEND
void ssv6xxx_early_suspend(struct early_suspend *h)
{
    //struct ssv_softc *sc = container_of(h, struct ssv_softc, early_suspend);
    //sc->ps_status = PWRSV_PREPARE;
    printk(KERN_INFO "ssv6xxx_early_suspend\n");
}


void ssv6xxx_late_resume(struct early_suspend *h)
{
    //struct ssv_softc *sc = container_of(h, struct ssv_softc, early_suspend);
    //if(sc)
        //ssv6xxx_disable_ps(sc);
    //else
        //printk(KERN_INFO "ssv6xxx_late_resume,sc=NULL\n");
    
    printk(KERN_INFO "ssv6xxx_late_resume\n");
}
#endif //CONFIG_HAS_EARLYSUSPEND
#endif // CONFIG_SSV_SUPPORT_ANDROID


void ssv6200_set_ps_work(struct work_struct *work)
{
    struct ssv_softc *sc = 
            container_of(work, struct ssv_softc, set_ps_work);

    ssv6xxx_ps_callback_func((unsigned long)sc);

}


static int ssv6200_config(struct ieee80211_hw *hw, u32 changed)
{
    struct ssv_softc *sc=hw->priv;
    int ret=0;

//    printk("%s(): changed: 0x%08x\n", __FUNCTION__, changed);
    mutex_lock(&sc->mutex);

#ifdef CONFIG_SSV_SUPPORT_ANDROID    
    if (changed & IEEE80211_CONF_CHANGE_PS) {
        struct ieee80211_conf *conf = &hw->conf;
        if (conf->flags & IEEE80211_CONF_PS) {
            if((sc->ps_aid)&&(sc->bStaPS == true)){
                //printk(KERN_INFO "Uk\n");
                wake_unlock(&sc->wifi_alive_lock);                
            }
        }else{
            if((sc->ps_aid)&&(sc->bStaPS == true)){
                //printk(KERN_INFO "Lk\n");
                wake_lock(&sc->wifi_alive_lock);
            }
        }
    }
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    if (changed & IEEE80211_CONF_CHANGE_QOS) {
        struct ieee80211_conf *conf = &hw->conf;
        bool qos_active = !!(conf->flags & IEEE80211_CONF_QOS);

        //set QoS status
        SMAC_REG_SET_BITS(sc->sh, ADR_GLBLE_SET, 
            (qos_active<<QOS_EN_SFT), QOS_EN_MSK);     
    }    
#endif

    if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
        struct ieee80211_channel *chan;
        #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
        chan = hw->conf.channel;
        #else
        chan = hw->conf.chandef.chan;
        #endif
//        struct ieee80211_channel *curchan = hw->conf.channel;
//        printk("%s(): Set channel to %d (%d MHz), sc->ch=%d, (%s)\n", __FUNCTION__, curchan->hw_value+1, 
//            curchan->center_freq, sc->cur_channel->hw_value+1,
//        ((hw->conf.flags&IEEE80211_CONF_OFFCHANNEL)? "off channel": "on channel"));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                {
                    struct ieee80211_channel *curchan = hw->conf.channel;
                    
                    if(sc->bScanning == true && 
                        sc->channel_center_freq != curchan->center_freq && sc->isAssoc){
        
                        hw->conf.flags |= IEEE80211_CONF_OFFCHANNEL;
                    }
                    else{
        
                        hw->conf.flags &= ~IEEE80211_CONF_OFFCHANNEL;
                    }
                }
#endif

#ifdef CONFIG_P2P_NOA
        if(sc->p2p_noa.active_noa_vif){
            printk("NOA operating-active vif[%02x] skip scan\n", sc->p2p_noa.active_noa_vif);
            goto out;
        }
#endif
        
        /**
         * If the current channel is off channel, pause all tx queue except 
         * management queue.
         */        
        if (hw->conf.flags & IEEE80211_CONF_OFFCHANNEL)
        {
            // If AP mode is enabled, ignore off channel config.
            if (   (sc->ap_vif == NULL)
                || list_empty(&((struct ssv_vif_priv_data *)sc->ap_vif->drv_priv)->sta_list))
            {
                // printk("off channel setting %p!\n", sc->ap_vif);
                HCI_PAUSE(sc->sh, (TXQ_EDCA_0|TXQ_EDCA_1|TXQ_EDCA_2|TXQ_EDCA_3));
                sc->sc_flags |= SC_OP_OFFCHAN;
            
                ssv6xxx_set_channel(sc, chan->hw_value);
                sc->hw_chan = chan->hw_value;
            }
            else
            {
                dev_dbg(sc->dev, "Off-channel to %d is ignored when AP mode enabled.\n", chan->hw_value);
            }
            /* if it is not connect to any device, let it takes more time to stay in a channel */
//            if(!sc->isAssoc)
//                msleep(800);
        }
        else {
            if (   (sc->cur_channel == NULL)
                || (sc->sc_flags & SC_OP_OFFCHAN)
                || (sc->hw_chan != chan->hw_value))
            {
                ssv6xxx_set_channel(sc, chan->hw_value);
                sc->cur_channel = chan;
                HCI_RESUME(sc->sh, (TXQ_EDCA_0|TXQ_EDCA_1|TXQ_EDCA_2|TXQ_EDCA_3));
                sc->sc_flags &= ~SC_OP_OFFCHAN;
            }
            else
            {
                dev_dbg(sc->dev, "Change to the same channel %d\n", chan->hw_value);
            }
            // printk("on channel setting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: ch=%d\n", sc->cur_channel->hw_value);
        }
    }
#ifdef CONFIG_P2P_NOA
out:
#endif
    mutex_unlock(&sc->mutex);
    return ret;
}


#if 0
static int sv6200_conf_tx(struct ieee80211_hw *hw,
                             struct ieee80211_vif *vif, u16 queue,
                             const struct ieee80211_tx_queue_params *params)
{
    struct ssv_softc *sc = hw->priv;
    u32 cw;
    u8 hw_txqid = sc->tx.hw_txqid[queue]; 

    printk("[I] sv6200_conf_tx qos[%d] queue[%d] aifsn[%d] cwmin[%d] cwmax[%d] txop[%d] \n",
    vif->bss_conf.qos, queue, params->aifs, params->cw_min, params->cw_max, params->txop);

    if (queue > NL80211_TXQ_Q_BK)
        return 1;
    mutex_lock(&sc->mutex);


    //set QoS status
    #define QOS_EN_MSK                            0x00000010
    #define QOS_EN_I_MSK                          0xffffffef
    #define QOS_EN_SFT                            4         
    #define QOS_EN_HI                             4         
    #define QOS_EN_SZ                             1
    SMAC_REG_SET_BITS(sc->sh, ADR_GLBLE_SET, (vif->bss_conf.qos<<QOS_EN_SFT), QOS_EN_MSK);


    //set wmm parameter
    cw = params->aifs&0xf;
    cw|= ((ilog2(params->cw_min+1))&0xf)<<8;
    cw|= ((ilog2(params->cw_max+1))&0xf)<<12;
    cw|= ((params->txop)&0xff)<<16;

    SMAC_REG_WRITE(sc->sh, ADR_TXQ0_MTX_Q_AIFSN+0x100*hw_txqid, cw);

    mutex_unlock(&sc->mutex);
    return 0;
}

#endif


#define SUPPORTED_FILTERS            \
    (FIF_PROMISC_IN_BSS |            \
    FIF_ALLMULTI |                   \
    FIF_CONTROL |                    \
    FIF_PSPOLL |                     \
    FIF_OTHER_BSS |                  \
    FIF_BCN_PRBRESP_PROMISC |        \
    FIF_PROBE_REQ |                  \
    FIF_FCSFAIL)

static void ssv6200_config_filter(struct ieee80211_hw *hw,
    unsigned int changed_flags,
    unsigned int *total_flags,
    u64 multicast)
{
//    struct ssv_softc *sc=hw->priv;
//
//    printk("%s(): changed_flags: 0x%08x, total_flags: 0x%08x\n", 
//    __FUNCTION__, changed_flags, *total_flags);
//
//    mutex_lock(&sc->mutex);

    /**
    * Note ??????????????
    * Modify this flag for AP mode ?????????
    */
    changed_flags &= SUPPORTED_FILTERS;
    *total_flags &= SUPPORTED_FILTERS;


//    mutex_unlock(&sc->mutex);

}


static void ssv6200_bss_info_changed(struct ieee80211_hw *hw,
        struct ieee80211_vif *vif, struct ieee80211_bss_conf *info,
        u32 changed)
{
    struct ssv_vif_priv_data *priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;
    struct ssv_softc *sc = hw->priv;
#ifdef CONFIG_P2P_NOA
    u8 null_address[6]={0};
#endif

//    printk("[I] %s(): VIF[%d] changed 0x[%08x]\n", __FUNCTION__, priv_vif->vif_idx, changed);

    mutex_lock(&sc->mutex);

//temp mark: RATE CTRL
    if (changed & BSS_CHANGED_ERP_PREAMBLE) {
        printk("BSS Changed use_short_preamble[%d]\n", info->use_short_preamble);
        if (info->use_short_preamble)
            sc->sc_flags |= SC_OP_SHORT_PREAMBLE;
        else
            sc->sc_flags &= ~SC_OP_SHORT_PREAMBLE;
    }

    /*set to hw if it's first vif config*/
    if (!priv_vif->vif_idx)
    {
        if (changed & BSS_CHANGED_BSSID) 
        {
#ifdef CONFIG_P2P_NOA        
            struct ssv_vif_priv_data *vif_priv;
            vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;
#endif
            
            /* Set BSSID to hardware and enable WSID entry 0 */
    		ssv62xxx_set_bssid(sc, (u8*)info->bssid);		

    	    printk("BSS_CHANGED_BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n",
            info->bssid[0], info->bssid[1], info->bssid[2],
            info->bssid[3], info->bssid[4], info->bssid[5]);

#ifdef CONFIG_P2P_NOA
            if(memcmp(info->bssid, null_address, 6))
                ssv6xxx_noa_hdl_bss_change(sc, MONITOR_NOA_CONF_ADD, vif_priv->vif_idx);
            else
                ssv6xxx_noa_hdl_bss_change(sc, MONITOR_NOA_CONF_REMOVE, vif_priv->vif_idx);
#endif                        
        }

        if (changed & BSS_CHANGED_ERP_SLOT)
        {
            u32 regval=0;

            printk("BSS_CHANGED_ERP_SLOT: use_short_slot[%d]\n", info->use_short_slot);
            if (info->use_short_slot)
            {
                SMAC_REG_READ(sc->sh, ADR_MTX_DUR_SIFS_G, &regval);
#if 1 
/*
    Fix MAC TX backoff issue.
    http://192.168.1.30/mantis/view.php?id=36
*/
                regval = regval & MTX_DUR_BURST_SIFS_G_I_MSK;
                regval |= 0xa << MTX_DUR_BURST_SIFS_G_SFT;
#endif
                regval = regval & MTX_DUR_SLOT_G_I_MSK;
                regval |= 9 << MTX_DUR_SLOT_G_SFT;
                SMAC_REG_WRITE(sc->sh, ADR_MTX_DUR_SIFS_G, regval);
                //slottime = 9;
            }
            else
            {
                SMAC_REG_READ(sc->sh, ADR_MTX_DUR_SIFS_G, &regval);
#if 1
/*
    Fix MAC TX backoff issue.
    http://192.168.1.30/mantis/view.php?id=36
*/
                regval = regval & MTX_DUR_BURST_SIFS_G_I_MSK;
                regval |= 0xa << MTX_DUR_BURST_SIFS_G_SFT;
#endif
                regval = regval & MTX_DUR_SLOT_G_I_MSK;
                regval |= 20 << MTX_DUR_SLOT_G_SFT;
                SMAC_REG_WRITE(sc->sh, ADR_MTX_DUR_SIFS_G, regval);
                //slottime = 20;
            }
        }
    }

    if (changed & BSS_CHANGED_HT) {
        printk("BSS_CHANGED_HT: Untreated!!\n");
    }


//temp mark: RATE CTRL
    if (changed & BSS_CHANGED_BASIC_RATES)
    {
        printk("ssv6xxx_rc_update_basic_rate!!\n");
        ssv6xxx_rc_update_basic_rate(sc, info->basic_rates);
    }

    if (vif->type == NL80211_IFTYPE_STATION){
        printk("NL80211_IFTYPE_STATION!!\n");
        if (changed & BSS_CHANGED_ASSOC){
            sc->isAssoc = info->assoc;
            if(!sc->isAssoc){
                sc->channel_center_freq = 0;
                sc->ps_aid = 0;
            }
            else{
                struct ieee80211_channel *curchan;
                #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
                curchan = hw->conf.channel;
                #else
                curchan = hw->conf.chandef.chan;
                #endif
                sc->channel_center_freq = curchan->center_freq;
                printk(KERN_INFO "!!info->aid = %d\n",info->aid);
                sc->ps_aid = info->aid;
            }
        }
    }

//--------------------------------------------------------------
    if (vif->type == NL80211_IFTYPE_AP)
    {
        if (changed & (  BSS_CHANGED_BEACON
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
                       | BSS_CHANGED_SSID
#endif                
                       | BSS_CHANGED_BSSID
                       | BSS_CHANGED_BASIC_RATES))
        {
#ifdef BROADCAST_DEBUG
             printk("[A] ssv6200_bss_info_changed:beacon changed\n");
#endif
             queue_work(sc->config_wq, &sc->set_tim_work); //ssv6xxx_beacon_change(sc, hw, vif, sc->aid0_bit_set);
        }
    
        if (changed & BSS_CHANGED_BEACON_INT)
        {
            printk("[A] BSS_CHANGED_BEACON_INT beacon_interval(%d)\n", info->beacon_int);
            if (sc->beacon_interval != info->beacon_int)
            {
                sc->beacon_interval = info->beacon_int;
                ssv6xxx_beacon_set_info(sc, sc->beacon_interval, sc->beacon_dtim_cnt);
            }
            
        }

        if (changed & BSS_CHANGED_BEACON_ENABLED)
        {
#ifdef BEACON_DEBUG            
            printk("[A] BSS_CHANGED_BEACON_ENABLED (0x%x)\n", info->enable_beacon);
#endif            
            if (0 != ssv6xxx_beacon_enable(sc, info->enable_beacon))
            {
                dev_err(sc->dev, "Beacon enable %d error.\n", info->enable_beacon);
            }
        }
    }

    mutex_unlock(&sc->mutex);
	printk("[I] %s(): leave\n", __FUNCTION__);
}


static int ssv6200_sta_add(struct ieee80211_hw *hw,
                           struct ieee80211_vif *vif,
                           struct ieee80211_sta *sta)
{
    struct ssv_sta_priv_data *sta_priv_dat=NULL;
    struct ssv_softc *sc=hw->priv;
    struct ssv_sta_info *sta_info;
    //u32 reg_val;
    u32 reg_wsid[] = {ADR_WSID0, ADR_WSID1};
    int s,i;
    u32 reg_wsid_tid0[] = {ADR_WSID0_TID0_RX_SEQ, ADR_WSID1_TID0_RX_SEQ};
    u32 reg_wsid_tid7[] = {ADR_WSID0_TID7_RX_SEQ, ADR_WSID1_TID7_RX_SEQ};
    unsigned long flags;
    int ret = 0;
    struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;
#ifdef FW_WSID_WATCH_LIST
    int fw_sec_caps = SSV6XXXX_WSID_SEC_NONE;
#endif
	printk("[I] %s(): vif[%d] ", __FUNCTION__, vif_priv->vif_idx);

    if (sc->force_triger_reset == true)
    {
        vif_priv->sta_asleep_mask = 0;
        do {
            spin_lock_irqsave(&sc->ps_state_lock, flags);
        /**
            * Add the new added station into the driver data sturcture.
            * Driver keeps this information for AMPDU use.
            * Note that we use drv_priv of struct ieee80211_sta to
            * bind our driver's sta structure.
            */
            for (s=0; s<SSV_NUM_STA; s++, sta_info++)
            {
                sta_info = &sc->sta_info[s];
                if ((sta_info->s_flags & STA_FLAG_VALID))
                {
                    if (sta_info->sta == sta)
                    {
                        printk("search stat %02x:%02x:%02x:%02x:%02x:%02x to  wsid=%d\n", 
                            sta->addr[0], sta->addr[1], sta->addr[2],
                            sta->addr[3], sta->addr[4], sta->addr[5], sta_info->hw_wsid);
                        spin_unlock_irqrestore(&sc->ps_state_lock, flags);
                        //sc->force_triger_reset = false;
                        return ret;
                    }
                }
            }
            spin_unlock_irqrestore(&sc->ps_state_lock, flags);

            if (s >= SSV_NUM_STA)
            {
                break;
            }
        } while(0);
    }

    do {
        spin_lock_irqsave(&sc->ps_state_lock, flags);
        // Freddie ToDo: Ensure sta_info sync to WSID.
        // 1. Find empty slot of sta_info for the STA.
        // 2. It's the WSID in single interface.
        // 3. In dual interface, if the same security mode is used. It's the WSID.
        //    WSID >= 2 should be set to firmware for WSID mapping in order to
        //    utilize hardware security decode.
        // 4. In dual interface, if different security mode is used,
        //    the first 2 WSID (HW) should be used by the primary interface.
        //    The second interface should not use HW WSID, and the WSID should
        //    not set to firmware in order to have software security work.
       /**
        * Add the new added station into the driver data structure.
        * Driver keeps this information for AMPDU use.
        * Note that we use drv_priv of struct ieee80211_sta to
        * bind our driver's sta structure.
        */
        //#ifndef FW_WSID_WATCH_LIST
        #if 1
        if (   (vif_priv->vif_idx == 0)
            || sc->sh->cfg.use_wpa2_only
           )
            s = 0;
        else
            s = 2;
        #else
        // Freddie ToDo:
        //   When security mode is the same: WSID 0 for primary interface. WSID 1 for second interface.
        //   Otherwise, WSID 0, 1 for primary. Secondary uses software security.
        #endif

        for (; s < SSV_NUM_STA; s++)
        {
            sta_info = &sc->sta_info[s];
            if ((sta_info->s_flags & STA_FLAG_VALID) == 0)
            {
                sta_info->aid = sta->aid;
                sta_info->sta = sta;
                sta_info->vif = vif;
                sta_info->s_flags = STA_FLAG_VALID;

                sta_priv_dat = 
                    (struct ssv_sta_priv_data *)sta->drv_priv;
                sta_priv_dat->sta_idx = s;
                sta_priv_dat->sta_info = sta_info;
                sta_priv_dat->has_hw_encrypt = false;
                sta_priv_dat->has_hw_decrypt = false;
                sta_priv_dat->need_sw_decrypt = false;
                sta_priv_dat->need_sw_encrypt = false;
                #ifdef USE_LOCAL_CRYPTO
                sta_priv_dat->crypt = NULL;
                sta_priv_dat->crypt_priv = NULL;
                #endif // USE_LOCAL_CRYPTO

                // WEP use single key for pairwise and broadcast frames.
                // In AP mode, key is only set when AP mode is initialized.
                if (   (vif_priv->pair_cipher == SSV_CIPHER_WEP40)
                    || (vif_priv->pair_cipher == SSV_CIPHER_WEP104))
                {
                    #ifdef USE_LOCAL_CRYPTO
                    if (vif_priv->crypt != NULL) 
                    {
                        sta_priv_dat->crypt = vif_priv->crypt;
                        sta_priv_dat->crypt_priv = vif_priv->crypt_priv;
                    }
                    #endif // USE_LOCAL_CRYPTO
                    sta_priv_dat->has_hw_encrypt = vif_priv->has_hw_encrypt;
                    sta_priv_dat->has_hw_decrypt = vif_priv->has_hw_decrypt;
                    sta_priv_dat->need_sw_encrypt = vif_priv->need_sw_encrypt;
                    sta_priv_dat->need_sw_decrypt = vif_priv->need_sw_decrypt;                    
                }

                list_add_tail(&sta_priv_dat->list, &vif_priv->sta_list);

//temp mark: PS
//                sc->ps_aid = sta->aid;
                break;
            }
        }

        spin_unlock_irqrestore(&sc->ps_state_lock, flags);

        if (s >= SSV_NUM_STA)
        {
            dev_err(sc->dev, "Number of STA exceeds driver limitation %d\n.", SSV_NUM_STA);
            ret = -1;
            break;
        }

        #ifdef CONFIG_SSV6XXX_DEBUGFS
        ssv6xxx_debugfs_add_sta(sc, sta_info);
        #endif // CONFIG_SSV6XXX_DEBUGFS

//-------------------------------------------------------------------------
//HW resource(!!! this section should put in the last. !!!)

        /* Set WSID to default*/
        sta_info->hw_wsid = -1;  

        /**
         * Allocate a free hardware WSID for the added STA. If no more
         * hardware entry present, set hw_wsid=-1 for
         * struct ssv_sta_info.
         */
        if (sta_priv_dat->sta_idx < SSV_NUM_HW_STA) 
        {
            #if 0
            /*  Disable to check vaild bit for station in ps mode
             *  When station in sleep mode, we neet to keep 
             *     hw register value to receive beacon. 
             *   Therefore host wakeup by AISC and wnat to add new station. 
             *   It may get valid bit is enable.
             */
            SMAC_REG_READ(sc->sh, reg_wsid[s], &reg_val);
            if ((reg_val & 0x01) == 0) 
            {
            #endif // 0

            /* Add STA into hardware for hardware offload */
            SMAC_REG_WRITE(sc->sh, reg_wsid[s]+4, *((u32 *)&sta->addr[0]));
            SMAC_REG_WRITE(sc->sh, reg_wsid[s]+8, *((u32 *)&sta->addr[4]));

            /* Valid this wsid entity */
            SMAC_REG_WRITE(sc->sh, reg_wsid[s], 1);
                
            /* Reset rx requence number */
            for (i = reg_wsid_tid0[s]; i <= reg_wsid_tid7[s]; i += 4)
                 SMAC_REG_WRITE(sc->sh, i, 0);
                
            /**
             * Enable hardware RC counters if the hardware RC is supported.
             * If RC is supported, the ssv6xxx_rate_alloc_sta() is called before
             * ssv6200_sta_add(). We shall make sure, if the added STA is 
             * set to SoC, we also enable RC for the STA.
             */
            ssv6xxx_rc_hw_reset(sc, sta_priv_dat->rc_idx, s);

            sta_info->hw_wsid = sta_priv_dat->sta_idx;
//              }
//              else BUG_ON(1);
        }
        #ifdef FW_WSID_WATCH_LIST
        else if (   (vif_priv->vif_idx == 0)
                 || sc->sh->cfg.use_wpa2_only
                )
        {
            sta_info->hw_wsid = sta_priv_dat->sta_idx;
        }
        #endif // FW_WSID_WATCH_LIST

#ifdef SSV6200_ECO
        // Freddie ToDo: When new STA is being added in AP mode,
        // apply key to corresponding key memory.
        if ((sta_priv_dat->has_hw_encrypt || sta_priv_dat->has_hw_decrypt) &&
            ((vif_priv->pair_cipher == SSV_CIPHER_WEP40) || (vif_priv->pair_cipher == SSV_CIPHER_WEP104)))
        {
            struct ssv_vif_info *vif_info = &sc->vif_info[vif_priv->vif_idx];
            struct ssv6xxx_hw_sec *sramKey = &vif_info->sramKey;

            //No matter current key is, update the pairwise key first
            _set_wep_hw_crypto_pair_key(sc, vif_info, sta_info, (void*)sramKey);
            if (sramKey->sta_key[0].pair_key_idx != 0)
            {
                // when current key is not pairwise key, update to the group key
                _set_wep_hw_crypto_group_key(sc, vif_info, sta_info, (void*)sramKey);
            }                            
        }
#endif // SSV6200_ECO
        ssv6200_ampdu_tx_add_sta(hw, sta);

        #ifdef FW_WSID_WATCH_LIST
        if (sta_info->hw_wsid >= SSV_NUM_HW_STA)
        {
            if (sta_priv_dat->has_hw_decrypt)
                // Only AP mode w. WEP security.
                fw_sec_caps = SSV6XXXX_WSID_SEC_PAIRWISE;
            
            if (vif_priv->need_sw_decrypt)
                fw_sec_caps |= SSV6XXXX_WSID_SEC_GROUP;
            
            hw_update_watch_wsid(sc, sta, sta_info, sta_priv_dat->sta_idx,
                fw_sec_caps, SSV6XXXX_WSID_OPS_ADD);
        }
        #endif // FW_WSID_WATCH_LIST

        printk("Add %02x:%02x:%02x:%02x:%02x:%02x to VIF %d sw_idx=%d, wsid=%d\n",
               sta->addr[0], sta->addr[1], sta->addr[2],
               sta->addr[3], sta->addr[4], sta->addr[5],
               vif_priv->vif_idx,
               sta_priv_dat->sta_idx, sta_info->hw_wsid);
    } while (0);

#ifdef CONFIG_SSV_SUPPORT_ANDROID
    if((sc->bStaPS == false)&&(vif->type == NL80211_IFTYPE_STATION)){
        printk(KERN_INFO "%s,init wifi_alive_lock\n",__FUNCTION__);
        wake_lock_init(&sc->wifi_alive_lock, WAKE_LOCK_SUSPEND, "wifi alive");
        sc->bStaPS = true;
    }
#endif

    return ret;
}


static int ssv6200_sta_remove(struct ieee80211_hw *hw,
   struct ieee80211_vif *vif,
   struct ieee80211_sta *sta)
{
    u32                         reg_wsid[] = {ADR_WSID0, ADR_WSID1};
    struct ssv_sta_priv_data   *sta_priv_dat = (struct ssv_sta_priv_data *)sta->drv_priv;
    struct ssv_softc           *sc = hw->priv;
    struct ssv_sta_info        *sta_info = sta_priv_dat->sta_info;
    unsigned long               flags;
    u32                         bit;
    struct ssv_vif_priv_data   *priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;
    u8                          hw_wsid = -1;

    BUG_ON(sta_priv_dat->sta_idx >= SSV_NUM_STA);

    dev_notice(sc->dev,
               "Removing STA %d (%02X:%02X:%02X:%02X:%02X:%02X) from VIF %d\n.",
               sta_priv_dat->sta_idx, sta->addr[0], sta->addr[1], sta->addr[2],
               sta->addr[3], sta->addr[4], sta->addr[5], priv_vif->vif_idx);

    spin_lock_irqsave(&sc->ps_state_lock, flags);

    #ifdef CONFIG_SSV6XXX_DEBUGFS
    // Remove STA debugfs during sta remove callback could results in deadlock.
    //ssv6xxx_debugfs_remove_sta(sc, sta_info);
    #endif // CONFIG_SSV6XXX_DEBUGFS

    #ifdef USE_LOCAL_CRYPTO
    //sta_priv_dat->KeySelect = 0;
    if (sta_priv_dat->crypt)
    {
        // If sta shares crypto with vif, deinit crypto when vif is being removed.
        if (   (priv_vif->crypt != sta_priv_dat->crypt)
            && sta_priv_dat->crypt_priv)
        {
            sta_priv_dat->crypt->deinit(sta_priv_dat->crypt_priv);
            dev_info(sc->dev, "STA releases crypto OK!\n");
        }
        sta_priv_dat->crypt_priv = NULL;
        sta_priv_dat->crypt = NULL;
    }
    #endif // USE_LOCAL_CRYPTO

    // Freddie ToDo: Remove when power saving?
    if ((sc->ps_status == PWRSV_PREPARE)||(sc->ps_status == PWRSV_ENABLE)) {
        //ssv6xxx_enable_ps(sc);
        memset(sta_info, 0, sizeof(*sta_info));
        sta_priv_dat->sta_idx = -1;

        list_del(&sta_priv_dat->list);

        spin_unlock_irqrestore(&sc->ps_state_lock, flags);
        return 0;
    }
    
    /*remove this sleep bit*/
    bit = BIT(sta_priv_dat->sta_idx);
    priv_vif->sta_asleep_mask &= ~bit;

    /* Remove invalid wsid entry */
    if (sta_info->hw_wsid != -1) {
#ifndef FW_WSID_WATCH_LIST
        BUG_ON(sta_info->hw_wsid >= SSV_NUM_HW_STA);
#endif
        hw_wsid = sta_info->hw_wsid;
    }
#ifdef FW_WSID_WATCH_LIST
    if (sta_info->hw_wsid >= SSV_NUM_HW_STA)
    {                
        spin_unlock_irqrestore(&sc->ps_state_lock, flags);
        hw_update_watch_wsid(sc, sta, sta_info, sta_info->hw_wsid, 0, SSV6XXXX_WSID_OPS_DEL);
        spin_lock_irqsave(&sc->ps_state_lock, flags);
    }
#endif

#if 0    
    printk("%s(): sw_idx=%d, hw_idx=%d sta_asleep_mask[%08x]\n", __FUNCTION__,
        sta_priv_dat->sta_idx , sta_info->hw_wsid, sc->sta_asleep_mask);
    printk("Remove %02x:%02x:%02x:%02x:%02x:%02x to sw_idx=%d, wsid=%d\n", 
    sta->addr[0], sta->addr[1], sta->addr[2],
    sta->addr[3], sta->addr[4], sta->addr[5], sta_priv_dat->sta_idx, sta_info->hw_wsid); 
#endif

    #ifdef CONFIG_SSV6XXX_DEBUGFS
    {
    // Remove STA debugfs during sta remove callback could results in deadlock.
    ssv6xxx_debugfs_remove_sta(sc, sta_info);
    }
    #endif // CONFIG_SSV6XXX_DEBUGFS

    memset(sta_info, 0, sizeof(*sta_info));
    sta_priv_dat->sta_idx = -1;
    list_del(&sta_priv_dat->list);
    if (list_empty(&priv_vif->sta_list) && vif->type == NL80211_IFTYPE_STATION)
    {
        priv_vif->pair_cipher = 0;
        priv_vif->group_cipher = 0;
    }

    spin_unlock_irqrestore(&sc->ps_state_lock, flags);

#if 0
    /**
        * Remove the specified station from the driver data structure.
        * Driver keeps this information for AMPDU use.
        */
    sta_info = sc->sta_info;
    for(s=0; s<SSV_NUM_STA; s++, sta_info++) {
        if (sta_info->s_flags & STA_FLAG_VALID)
            continue;
        if (sta_info->sta == sta && 
            sta_info->vif == vif)
    sta_info->s_flags = 0;
    }
#endif

#ifndef FW_WSID_WATCH_LIST
    if(hw_wsid != -1)
#else
    if((hw_wsid != -1) && (hw_wsid < SSV_NUM_HW_STA))
#endif
        SMAC_REG_WRITE(sc->sh, reg_wsid[hw_wsid], 0x00);
    
#ifdef CONFIG_SSV_SUPPORT_ANDROID
    if(sc->bStaPS == true){
        wake_lock_destroy(&sc->wifi_alive_lock);
        printk(KERN_INFO "%s,destroy wifi_alive_lock\n",__FUNCTION__);
        sc->bStaPS = false;
    }
#endif // CONFIG_SSV_SUPPORT_ANDROID
    return 0;
}


static void ssv6200_sta_notify(struct ieee80211_hw *hw,
                                 struct ieee80211_vif *vif,
                                 enum sta_notify_cmd cmd,
                                 struct ieee80211_sta *sta)
{
    struct ssv_softc *sc = hw->priv;
    struct ssv_vif_priv_data *priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;
    struct ssv_sta_priv_data *sta_priv_dat = (struct ssv_sta_priv_data *)sta->drv_priv;
    struct ssv_sta_info *sta_info;

    u32 bit, prev;
    unsigned long flags;

#ifdef BROADCAST_DEBUG
//    printk("[I] %s(): =>\n", __FUNCTION__);
#endif

    spin_lock_irqsave(&sc->ps_state_lock, flags);
    
    bit = BIT(sta_priv_dat->sta_idx);
    
    /* check if STA has already marked to sleep mode*/
	prev = priv_vif->sta_asleep_mask & bit;
	sta_info = sta_priv_dat->sta_info;
    switch (cmd)
    {
        case STA_NOTIFY_SLEEP:
            if(!prev)
            {
                sta_info->sleeping = true;

                if (   (vif->type == NL80211_IFTYPE_AP)
                    && sc->bq4_dtim 
                    && !priv_vif->sta_asleep_mask 
                    && ssv6200_bcast_queue_len(&sc->bcast_txq)){
//#ifdef BROADCAST_DEBUG
                    printk("%s(): ssv6200_bcast_start\n", __FUNCTION__);
//#endif
                    ssv6200_bcast_start(sc);
                }

                /* mark to sleep mode */
				priv_vif->sta_asleep_mask |= bit;
            }
            break;
        case STA_NOTIFY_AWAKE:
            if(prev)
            {
                sta_info->sleeping = false;

                /* STA awake remove this bit*/
				priv_vif->sta_asleep_mask &= ~bit;

//                if (!sc->sta_asleep_mask){
//#ifdef BROADCAST_DEBUG
//                    printk("%s(): ssv6200_bcast_stop:\n", __FUNCTION__);                   
//#endif
//                    ssv6200_bcast_stop(sc);//?? it could not be called from here.
//                }
            }
            break;
        default:
            break;
    }
    
    spin_unlock_irqrestore(&sc->ps_state_lock, flags);
    
//#ifdef BROADCAST_DEBUG
//    printk("========================>%s:%02x:%02x:%02x:%02x:%02x:%02x\n",
//        (cmd==STA_NOTIFY_SLEEP)?"STA_NOTIFY_SLEEP":"STA_NOTIFY_AWAKE",
//        sta->addr[0], sta->addr[1], sta->addr[2],
//        sta->addr[3], sta->addr[4], sta->addr[5]);
//#endif
    
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)   
static u64 ssv6200_get_tsf(struct ieee80211_hw *hw)
#else
static u64 ssv6200_get_tsf(struct ieee80211_hw *hw, 
                           struct ieee80211_vif *vif)
#endif
{
    printk("%s(): \n", __FUNCTION__);
    return 0;
}


static void ssv6200_sw_scan_start(struct ieee80211_hw *hw)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)        
    ((struct ssv_softc *)(hw->priv))->bScanning = true;
#endif
    printk("--------------%s(): \n", __FUNCTION__);
}


static void ssv6200_sw_scan_complete(struct ieee80211_hw *hw)
{

    printk("==============%s(): \n", __FUNCTION__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    ((struct ssv_softc *)(hw->priv))->bScanning = false;
#endif    
}


static int ssv6200_set_tim(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
                            bool set)
{
    struct ssv_softc *sc = hw->priv;
//    int sta_idx = 0;
    struct ssv_sta_info *sta_info =   sta
                                    ? ((struct ssv_sta_priv_data *)sta->drv_priv)->sta_info
                                    : NULL;    
    
    /* Call set beacon if tim bit is changed */
    if (sta_info && (sta_info->tim_set^set))
    {
#ifdef BROADCAST_DEBUG
           printk("[I] [A] ssvcabrio_set_tim");
#endif           
        sta_info->tim_set = set;
        queue_work(sc->config_wq, &sc->set_tim_work);
    }
    
    return 0;
}

      
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
static int ssv6200_conf_tx(struct ieee80211_hw *hw, u16 queue,
           const struct ieee80211_tx_queue_params *params)
#else
static int ssv6200_conf_tx(struct ieee80211_hw *hw,
                             struct ieee80211_vif *vif, u16 queue,
                             const struct ieee80211_tx_queue_params *params)
#endif
{
    struct ssv_softc *sc = hw->priv;
    u32 cw;
    u8 hw_txqid = sc->tx.hw_txqid[queue]; 
    struct ssv_vif_priv_data *priv_vif = (struct ssv_vif_priv_data *)vif->drv_priv;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
    printk("[I] sv6200_conf_tx vif[%d] qos[%d] queue[%d] aifsn[%d] cwmin[%d] cwmax[%d] txop[%d] \n",
		priv_vif->vif_idx ,vif->bss_conf.qos, queue, params->aifs, params->cw_min, params->cw_max, params->txop);
#else
    printk("[I] sv6200_conf_tx queue[%d] aifsn[%d] cwmin[%d] cwmax[%d] txop[%d] \n",
        queue, params->aifs, params->cw_min, params->cw_max, params->txop);
#endif

    if (queue > NL80211_TXQ_Q_BK)
        return 1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)    
    if (priv_vif->vif_idx != 0) {
        dev_warn(sc->dev, "WMM setting applicable to primary interface only.\n");
        return 1;
    }
#endif

    mutex_lock(&sc->mutex);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
    //set QoS status
    SMAC_REG_SET_BITS(sc->sh, ADR_GLBLE_SET, 
            (vif->bss_conf.qos<<QOS_EN_SFT), QOS_EN_MSK);
#endif

#if 0
	//Fix backoff
    {
        cw = 0x4;
        SMAC_REG_WRITE(sc->sh, ADR_TXQ0_MTX_Q_MISC_EN+0x100*hw_txqid, cw);
        cw = 0x0;
        SMAC_REG_WRITE(sc->sh, ADR_TXQ0_MTX_Q_BKF_CNT+0x100*hw_txqid, cw);
    }
#endif

	//set wmm parameter
#if 1
/*
    Fix MAC TX backoff issue.
    http://192.168.1.30/mantis/view.php?id=36
 */
    cw = (params->aifs-1)&0xf;
#else
    cw = params->aifs&0xf;
#endif
    cw|= ((ilog2(params->cw_min+1))&0xf)<<TXQ1_MTX_Q_ECWMIN_SFT;//8;
    cw|= ((ilog2(params->cw_max+1))&0xf)<<TXQ1_MTX_Q_ECWMAX_SFT;//12;
    cw|= ((params->txop)&0xff)<<TXQ1_MTX_Q_TXOP_LIMIT_SFT;//16;

    SMAC_REG_WRITE(sc->sh, ADR_TXQ0_MTX_Q_AIFSN+0x100*hw_txqid, cw);

    mutex_unlock(&sc->mutex);
    return 0;
}




#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
static int ssv6200_ampdu_action(struct ieee80211_hw *hw,
              struct ieee80211_vif *vif,
              enum ieee80211_ampdu_mlme_action action,
              struct ieee80211_sta *sta,
              u16 tid, u16 *ssn)

#else
static int ssv6200_ampdu_action(struct ieee80211_hw *hw,
              struct ieee80211_vif *vif,
              enum ieee80211_ampdu_mlme_action action,
              struct ieee80211_sta *sta,
              u16 tid, u16 *ssn, u8 buf_size)
#endif


{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    u8 buf_size = 32;
#endif
    struct ssv_softc *sc = hw->priv;
    int ret = 0;
    //printk("[I] %s(): \n", __FUNCTION__);

    if(sta == NULL)
        return ret;


    #if (!Enable_AMPDU_Rx)
    if(action == IEEE80211_AMPDU_RX_START || action == IEEE80211_AMPDU_RX_STOP )
    {
        ampdu_db_log("Disable AMPDU_RX for test(1).\n");
        return -EOPNOTSUPP;
    }
    #endif
    
    #if (!Enable_AMPDU_Tx)
    if(action == IEEE80211_AMPDU_TX_START || action == IEEE80211_AMPDU_TX_STOP || action == IEEE80211_AMPDU_TX_OPERATIONAL )
    {
        ampdu_db_log("Disable AMPDU_TX for test(1).\n");
        return -EOPNOTSUPP;
    }
    #endif

    if((action == IEEE80211_AMPDU_RX_START || action == IEEE80211_AMPDU_RX_STOP ) &&
        (!(sc->sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_RX)))
    {
        ampdu_db_log("Disable AMPDU_RX(2).\n");
        return -EOPNOTSUPP;
    }

    if(   (   action == IEEE80211_AMPDU_TX_START
           #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
           || action == IEEE80211_AMPDU_TX_STOP
           #else
           || action == IEEE80211_AMPDU_TX_STOP_CONT
           || action == IEEE80211_AMPDU_TX_STOP_FLUSH
           || action == IEEE80211_AMPDU_TX_STOP_FLUSH_CONT
           #endif
           || action == IEEE80211_AMPDU_TX_OPERATIONAL )
       && (!(sc->sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_TX)))
    {
        ampdu_db_log("Disable AMPDU_TX(2).\n");
        return -EOPNOTSUPP;
    }

    //mutex_lock(&sc->mutex);
    switch (action)
    {
        case IEEE80211_AMPDU_RX_START:
#ifdef WIFI_CERTIFIED
            if (sc->rx_ba_session_count >= SSV6200_RX_BA_MAX_SESSIONS)
            {
                /* Workaround solution:
				   ASIC just support one AMPDU RX 
				   BA session, not partial state. 
				   Need to cancel previous BA session
				*/					
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
                ieee80211_stop_rx_ba_session(vif,
						     (1<<(sc->ba_tid)),
						     sc->ba_ra_addr);
#endif
                sc->rx_ba_session_count--;
            }
#else
            /*
                Fix mantis issue 41 & 43.
                (AP mode)Repeatability of BA request can cause problems with the connection.
            */
            if ((sc->rx_ba_session_count >= SSV6200_RX_BA_MAX_SESSIONS) && (sc->rx_ba_sta != sta))
            {
                ret = -EBUSY;
                break;
            }
            else if ((sc->rx_ba_session_count >= SSV6200_RX_BA_MAX_SESSIONS) && (sc->rx_ba_sta == sta))
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
                ieee80211_stop_rx_ba_session(vif,(1<<(sc->ba_tid)),sc->ba_ra_addr);
#endif
                sc->rx_ba_session_count--;
            }
#endif
            printk(KERN_ERR "IEEE80211_AMPDU_RX_START %02X:%02X:%02X:%02X:%02X:%02X %d.\n",
                   sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3],
                   sta->addr[4], sta->addr[5], tid);

            sc->rx_ba_session_count++;

            sc->rx_ba_sta = sta;
            sc->ba_tid = tid;
            sc->ba_ssn = *ssn;
            memcpy(sc->ba_ra_addr, sta->addr, ETH_ALEN);
            queue_work(sc->config_wq, &sc->set_ampdu_rx_add_work);

            //Set register
            //ssv6200_hw_set_rx_ba_session(sc->sh, true, sta->addr, tid, *ssn, buf_size);

            break;

        case IEEE80211_AMPDU_RX_STOP:
            sc->rx_ba_session_count--;
            if (sc->rx_ba_session_count == 0)
                sc->rx_ba_sta = NULL;

            queue_work(sc->config_wq, &sc->set_ampdu_rx_del_work);
            //Clear register
            //ssv6200_hw_set_rx_ba_session(sc->sh, false, sta->addr, tid, 0, 0);

            break;
            
        case IEEE80211_AMPDU_TX_START:
            printk(KERN_ERR "AMPDU_TX_START %02X:%02X:%02X:%02X:%02X:%02X %d.\n",
                   sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3],
                   sta->addr[4], sta->addr[5], tid);
            ssv6200_ampdu_tx_start(tid, sta, hw, ssn);
            ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
            break;
            
        #if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
        case IEEE80211_AMPDU_TX_STOP:
        #else
            /* @IEEE80211_AMPDU_TX_STOP_CONT: stop TX aggregation but continue transmitting
             *  queued packets, now unaggregated. After all packets are transmitted the
             *  driver has to call ieee80211_stop_tx_ba_cb_irqsafe().
             * @IEEE80211_AMPDU_TX_STOP_FLUSH: stop TX aggregation and flush all packets,
             *  called when the station is removed. There's no need or reason to call
             *  ieee80211_stop_tx_ba_cb_irqsafe() in this case as mac80211 assumes the
             *  session is gone and removes the station.
             * @IEEE80211_AMPDU_TX_STOP_FLUSH_CONT: called when TX aggregation is stopped
             *  but the driver hasn't called ieee80211_stop_tx_ba_cb_irqsafe() yet and
             *  now the connection is dropped and the station will be removed. Drivers
             *  should clean up and drop remaining packets when this is called.
             */
        // Freddie ToDo:
        case IEEE80211_AMPDU_TX_STOP_CONT:
        case IEEE80211_AMPDU_TX_STOP_FLUSH:
        case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
        #endif
            printk(KERN_ERR "AMPDU_TX_STOP %02X:%02X:%02X:%02X:%02X:%02X %d.\n",
                   sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3],
                   sta->addr[4], sta->addr[5], tid);
            ssv6200_ampdu_tx_stop(tid, sta, hw);
            ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
            break;

        case IEEE80211_AMPDU_TX_OPERATIONAL:
            printk(KERN_ERR "AMPDU_TX_OPERATIONAL %02X:%02X:%02X:%02X:%02X:%02X %d.\n",
                   sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3],
                   sta->addr[4], sta->addr[5], tid);
            ssv6200_ampdu_tx_operation(tid, sta, hw, buf_size);
            break;
            
        default:
            ret = -EOPNOTSUPP;//not support.
            break;
    }
    
    //mutex_unlock(&sc->mutex);
    
    return ret;
}


struct ieee80211_ops ssv6200_ops =
{
    /* MUST callback function: */
    .tx                 = ssv6200_tx,
    .start              = ssv6200_start,
    .stop               = ssv6200_stop,
    .add_interface      = ssv6200_add_interface,
    .remove_interface   = ssv6200_remove_interface,
    .config             = ssv6200_config,
    .configure_filter   = ssv6200_config_filter,
    

    /* OPTIONAL callback function: */
    .bss_info_changed   = ssv6200_bss_info_changed,
    .sta_add            = ssv6200_sta_add,
    .sta_remove         = ssv6200_sta_remove,
    .sta_notify         = ssv6200_sta_notify,
    .set_key            = ssv6200_set_key,
    .sw_scan_start      = ssv6200_sw_scan_start,
    .sw_scan_complete   = ssv6200_sw_scan_complete,
    .get_tsf            = ssv6200_get_tsf,
    .set_tim            = ssv6200_set_tim,
    .conf_tx            = ssv6200_conf_tx,
    .ampdu_action       = ssv6200_ampdu_action,
};


// Freddie ToDo: local crypto should be initialzed by-key (by-STA/by-VIF).
// SKB then can be be encrypted correctly by its corresponding initialized
// crypto data in STA/VIF.
#ifdef USE_LOCAL_CRYPTO
#ifdef MULTI_THREAD_ENCRYPT
void ssv6xxx_skb_get_tx_cryptops(struct sk_buff *mpdu, struct ssv_crypto_ops **crypt_ptr, void **crypt_priv_ptr)
{
    struct ieee80211_tx_info    *info = IEEE80211_SKB_CB(mpdu);
    struct SKB_info_st          *skb_info = (struct SKB_info_st *)mpdu->head;
    struct ieee80211_sta        *sta = skb_info->sta;
    struct ieee80211_hdr        *hdr = (struct ieee80211_hdr *)mpdu->data;
    u32                          unicast = (is_multicast_ether_addr(hdr->addr1))? 0: 1;
    struct ssv_sta_priv_data    *sta_priv_dat = NULL;


    BUG_ON((size_t)info < (size_t)0x01000);

    if (unicast)
    {
        if (sta)
        {
            sta_priv_dat = (struct ssv_sta_priv_data *)sta->drv_priv;
            *crypt_ptr = sta_priv_dat->crypt;
            *crypt_priv_ptr = sta_priv_dat->crypt_priv;
        }
        else
        {
            printk(KERN_ERR
                   "Unicast to NULL STA frame. "
                   "%02X:%02X:%02X:%02X:%02X:%02X -> %02X:%02X:%02X:%02X:%02X:%02X)\n",
                   hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                   hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
                   hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
                   hdr->addr2[3], hdr->addr2[4], hdr->addr2[5]);
            *crypt_ptr = NULL;
            *crypt_priv_ptr = NULL;
        }
    }
    else
    {
        struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)info->control.vif->drv_priv;

        *crypt_ptr = vif_priv->crypt;
        *crypt_priv_ptr = vif_priv->crypt_priv;
    }
}


int ssv6xxx_skb_pre_encrypt(struct sk_buff *mpdu, struct ssv_softc *sc)
{
    struct ssv_crypto_ops *crypt = NULL;
    void *crypt_priv = NULL;
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)mpdu->data;
    int ret = -1;
    
    ssv6xxx_skb_get_tx_cryptops(mpdu, &crypt, &crypt_priv);
    
    if(crypt != NULL && crypt->encrypt_prepare != NULL)
    {
        u32 hdrlen = ieee80211_hdrlen(hdr->frame_control);
        ret = crypt->encrypt_prepare(mpdu, hdrlen, crypt_priv);        
    }
    else
    {
		 // Debug code
        /*struct SKB_info_st          *skb_info = (struct SKB_info_st *)mpdu->head;
        struct ieee80211_sta        *sta = skb_info->sta;
        u32                          unicast = (is_multicast_ether_addr(hdr->addr1)) ? 0: 1;

        if ((sta != NULL) || !unicast)
        {
            dev_err(sc->dev, "pre-encrypt %c for %02X:%02X:%02X:%02X:%02X:%02X with NULL crypto.\n",
                    (sta && unicast) ? 'U' : 'M',
                            sta->addr[0], sta->addr[1], sta->addr[2],
                            sta->addr[3], sta->addr[4], sta->addr[5]);
        }*/
        ret = -EOPNOTSUPP;
    }
    //printk("ssv6xxx_skb_pre_encrypt ret = %d\n", ret);
    return ret;
}

int ssv6xxx_skb_pre_decrypt(struct sk_buff *mpdu, struct ieee80211_sta *sta, struct ssv_softc *sc)
{
    struct ssv_crypto_ops *crypt = NULL;
    void *crypt_priv = NULL;
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)mpdu->data;
    int ret = -1;
    
    ssv6xxx_skb_get_rx_cryptops(mpdu, sta, sc, &crypt, &crypt_priv);
    
    if(crypt != NULL && crypt->decrypt_prepare != NULL)
    {
        u32 hdrlen = ieee80211_hdrlen(hdr->frame_control);
        ret = crypt->decrypt_prepare(mpdu, hdrlen, crypt_priv);        
    }
    else
    {
        //dev_err(sc->dev, "pre-decrypt with NULL crypto.\n");
        ret = -EOPNOTSUPP;
    }
    //printk("ssv6xxx_skb_pre_decrypt ret = %d\n", ret);
    return ret;
}

#endif

int ssv6xxx_skb_encrypt(struct sk_buff *mpdu, struct ssv_softc *sc)
{
    struct ieee80211_hdr           *hdr = (struct ieee80211_hdr *)mpdu->data;
	int ret = 0;
    u32                             unicast = (is_multicast_ether_addr(hdr->addr1))? 0: 1;
#ifndef MULTI_THREAD_ENCRYPT
    struct ieee80211_tx_info       *tx_info = IEEE80211_SKB_CB(mpdu);
    struct SKB_info_st             *skb_info = (struct SKB_info_st *)mpdu->head;
    struct ieee80211_sta           *sta = skb_info->sta;
    //u32 wsid=0;   

    struct ssv_sta_priv_data       *sta_priv_dat = NULL;

    //u8 *key, keyidx;
    //u32 kselect_ori=0;
#endif
    struct ssv_crypto_ops          *crypt = NULL;
    void                           *crypt_priv = NULL;
#ifndef MULTI_THREAD_ENCRYPT

    if (sta || unicast)
    {
        sta_priv_dat = (struct ssv_sta_priv_data *)sta->drv_priv;
        //wsid = (!sta_info || sta_info->hw_wsid<0) ? 0x0F : sta_info->hw_wsid;
        //kselect_ori = sta_info->KeySelect;
        //printk("ampdu encry: wsid=%d\n",wsid);
        crypt = sta_priv_dat->crypt;
        crypt_priv = sta_priv_dat->crypt_priv;
    }
    else
    // Crypto for non uni-cast frame is in VIF private data.
    //if ((unicast == 0) || (sta == NULL))
    {
        struct ssv_vif_priv_data *vif_priv = (struct ssv_vif_priv_data *)tx_info->control.vif->drv_priv;

        crypt = vif_priv->crypt;
        crypt_priv = vif_priv->crypt_priv;
    }
#else
    ssv6xxx_skb_get_tx_cryptops(mpdu, &crypt, &crypt_priv);
#endif
    //printk("mpdu encry: unicast=%d\n",unicast);

    if (crypt == NULL)
    {
        dev_err(sc->dev, "[Local Crypto]: Encrypt %c %d %02X:%02X:%02X:%02X:%02X:%02X with NULL crypto.\n",
                unicast ? 'U' : 'B', mpdu->protocol,
                hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
        return -1;
    }
    else
    {
        u32 hdrlen = ieee80211_hdrlen(hdr->frame_control);
        // Freddie ToDo: Handler encryption failure.
        ret = crypt->encrypt_mpdu(mpdu, hdrlen, crypt_priv);
    }
	return ret;
}

void ssv6xxx_skb_get_rx_cryptops(struct sk_buff *mpdu, struct ieee80211_sta *sta, struct ssv_softc *sc,
    struct ssv_crypto_ops **crypt_ptr, void **crypt_priv_ptr)
{
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)mpdu->data;
    struct ssv_sta_priv_data *sta_priv;
    struct ssv_vif_priv_data *vif_priv;
    struct ssv_vif_info  *vif_info;
    u32 unicast = 0;
    if(sta == NULL)
    {
        printk("No sta, fail to get rx cryptops\n");
        return;
    }
    
    sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
    vif_priv = (struct ssv_vif_priv_data *)sta_priv->sta_info->vif->drv_priv;
    vif_info = &sc->vif_info[vif_priv->vif_idx];
    if (vif_info->if_type == NL80211_IFTYPE_STATION)    
        unicast = (is_multicast_ether_addr(hdr->addr1))?0:1;
    
    //printk("[Local Crypto]: DEbug, sta = %p, unicast = %d, vif_info->if_type = %d\n", sta, unicast, vif_info->if_type);
    
    if((sta->drv_priv != NULL) && (vif_info->if_type == NL80211_IFTYPE_AP))
    {
        //printk("[Local Crypto]: ####### 1 \n");
        *crypt_ptr = sta_priv->crypt;
        *crypt_priv_ptr = sta_priv->crypt_priv;        
    }
    else if((sta->drv_priv != NULL) && (unicast == 1))
    {
        //printk("[Local Crypto]: ####### 2 \n");
        *crypt_ptr = sta_priv->crypt;
        *crypt_priv_ptr = sta_priv->crypt_priv;
    }
    else if((unicast != 1) && (vif_priv != NULL))
    {
        //printk("[Local Crypto]: ####### 3 \n");
        *crypt_ptr = vif_priv->crypt;
        *crypt_priv_ptr = vif_priv->crypt_priv;
    }
    else
    {
        printk("[Local Crypto]: No useful drv_priv, sta = %p, unicast = %d, vif_priv = %p", sta, unicast, vif_priv);
        if(sta != NULL)
            printk(", sta_priv = %p", sta->drv_priv);
        printk("\n");        
    }
}

int ssv6xxx_skb_decrypt(struct sk_buff *mpdu, struct ieee80211_sta *sta, struct ssv_softc *sc)
{
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)mpdu->data;
    struct ssv_crypto_ops          *crypt = NULL;
    void                           *crypt_priv = NULL;    
    u32 hdrlen = ieee80211_hdrlen(hdr->frame_control);
    
    ssv6xxx_skb_get_rx_cryptops(mpdu, sta, sc, &crypt, &crypt_priv);

    if ((crypt != NULL) && (crypt_priv != NULL))
        return (crypt->decrypt_mpdu(mpdu, hdrlen, crypt_priv));
    
	printk("[Local Crypto]: crytp is null\n");
    return -1;
}
#endif // USE_LOCAL_CRYPTO

int ssv6200_tx_flow_control(void *dev, int hw_txqid, bool fc_en,int debug)
{
    struct ssv_softc *sc=dev;
    int ac;

    BUG_ON(hw_txqid > 4);

    /**
        * No flow control for hardware queue 4 due to we use a 
        * hardware queue 4 for the management/NULL frames.
        */
    if (hw_txqid == 4)
        return 0;

    // Freddie ToDo: Use flow control counter instead of a single bit flag
    // to management flow control because more than one condition will
    // issue flow control.
    ac = sc->tx.ac_txqid[hw_txqid];
    if (fc_en == false) {
        /* disable tx flow control */
        if (sc->tx.flow_ctrl_status & (1<<ac)) {
                ieee80211_wake_queue(sc->hw, ac);
                //printk("TX Q%d O %d\n", hw_txqid, debug);
                sc->tx.flow_ctrl_status &= ~(1<<ac);
//                          printk("%s(): flow control OFF (flow_ctrl_status=%x, hw_txqid=%d, ac=%d)\n", 
//                          __FUNCTION__, sc->tx.flow_ctrl_status, hw_txqid, ac);  
        } else {
            //printk(KERN_ERR "Wake non-stopped TX Q #%d.\n", hw_txqid);
        }

    }
    else {
        /* enable tx flow control */
        if ((sc->tx.flow_ctrl_status & (1<<ac))==0) { 
            ieee80211_stop_queue(sc->hw, ac);
            //printk("TX Q%d X %d\n", hw_txqid, debug);
            sc->tx.flow_ctrl_status |= (1<<ac);
//                   printk("%s(): flow control ON (flow_ctrl_status=%x, hw_txqid=%d, ac=%d)\n", 
//                   __FUNCTION__, sc->tx.flow_ctrl_status, hw_txqid, ac);
        } else {
            //printk(KERN_ERR "Stop stopped TX Q #%d.\n", hw_txqid);
        }
    }
    return 0;
}


// HCI queue empty
void ssv6xxx_tx_q_empty_cb (u32 txq_no, void *cb_data)
{
    struct ssv_softc *sc = cb_data;
    // HCI queue is empty, wake up TX thread to flush buffered skb to HCI queue.
    BUG_ON(sc == NULL);
    sc->tx_q_empty = true;
    wake_up_interruptible(&sc->tx_wait_q);
} // end of - ssv6xxx_tx_q_empty -


//static int rx_count = 0;
// Process RX SKB, i.e. strip SSV header and appended data and send it to upper layer
static void _proc_data_rx_skb (struct ssv_softc *sc, struct sk_buff *rx_skb)
{
    struct ieee80211_rx_status             *rxs;
    struct ieee80211_hdr                   *hdr;
    __le16                                  fc;
    struct ssv6200_rx_desc                 *rxdesc;
    struct ssv6200_rxphy_info_padding      *rxphypad;
    struct ssv6200_rxphy_info              *rxphy;
    struct ieee80211_channel               *chan;
    struct ieee80211_vif                   *vif = NULL;
    struct ieee80211_sta                   *sta = NULL;
    bool                                    rx_hw_dec = false;
    bool                                    do_sw_dec = false;
    struct ssv_sta_priv_data               *sta_priv = NULL;
    struct ssv_vif_priv_data               *vif_priv = NULL;
	static int                              prev_rssi = 0;
    SKB_info *skb_info = NULL;
#ifdef USE_LOCAL_CRYPTO    
    int                                     ret = 0;
#ifdef MULTI_THREAD_ENCRYPT
    struct ssv_encrypt_task_list *ta = NULL;
    unsigned long flags;
#endif
#endif
    //unsigned long flags;

    /* extract headers */
    rxdesc = (struct ssv6200_rx_desc *)rx_skb->data;
    rxphy = (struct ssv6200_rxphy_info *)(rx_skb->data + sizeof(*rxdesc));
    rxphypad = (struct ssv6200_rxphy_info_padding *)(rx_skb->data + rx_skb->len - sizeof(struct ssv6200_rxphy_info_padding));

    hdr = (struct ieee80211_hdr *)(rx_skb->data + SSV6XXX_RX_DESC_LEN);
    fc = hdr->frame_control;
    skb_info = (SKB_info *)rx_skb->head;

    /*
        // Parser connected STA data rate(Rate control)
        WSID0 & WSID1 we use rate control alogrithm.
        WSID2 - WSID7 based on the other side of the data rate.
    */
    if (rxdesc->wsid >= SSV_RC_MAX_HARDWARE_SUPPORT)
    {
        if (   (ieee80211_is_data(hdr->frame_control))
            && (!(ieee80211_is_nullfunc(hdr->frame_control))))
        {
            ssv6xxx_rc_rx_data_handler(sc->hw, rx_skb, rxdesc->rate_idx);
        }
    }

    /* prepare rx status for mac80211 */
    rxs = IEEE80211_SKB_RXCB(rx_skb);
    memset(rxs, 0, sizeof(struct ieee80211_rx_status));
    ssv6xxx_rc_mac8011_rate_idx(sc, rxdesc->rate_idx, rxs);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    rxs->mactime = *((u32 *)&rx_skb->data[28]);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    chan = sc->hw->conf.channel;
#else
    chan = sc->hw->conf.chandef.chan;
#endif
    rxs->band = chan->band;
    rxs->freq = chan->center_freq;
    rxs->antenna = 1;
	
    //max:0 min:-127
    if (rxdesc->rate_idx < SSV62XX_G_RATE_INDEX && rxphypad->RSVD == 0)
	    rxs->signal = (-rxphypad->rpci);
    else if (rxdesc->rate_idx >= SSV62XX_G_RATE_INDEX && rxphy->service == 0)
	    rxs->signal = (-rxphy->rpci);
    else {

#if 0 /*For debug*/
	    printk("########unicast: %d, phy: %d, phypad: %d###############\n", 
						rxdesc->unicast, (-rxphy->rpci), (-rxphypad->rpci));
		printk("RSSI, %d, rate_idx, %d, prev_rssi, %d\n", rxs->signal, rxdesc->rate_idx, prev_rssi);
#endif
	    rxs->signal = prev_rssi;
    }	 
    prev_rssi = rxs->signal;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    rxs->flag = RX_FLAG_MACTIME_MPDU;
#else
    rxs->flag = RX_FLAG_MACTIME_START;
#endif
    rxs->rx_flags = 0;
#endif

#if LINUX_VERSION_CODE >= 0x030400
    if (rxphy->aggregate)
        rxs->flag |= RX_FLAG_NO_SIGNAL_VAL;
#endif

    /* update Mng queue status to let host know*/
    sc->hw_mng_used = rxdesc->mng_used;

    /*
     * Set protect bit of IEEE 802.11 data frame header  to 0 if
     * hardware security offload engine is enabled for security mode.
     */
    // Check decryption status of the protected data frame
    if (   (ieee80211_is_data(fc) || ieee80211_is_data_qos(fc))
        && ieee80211_has_protected(fc))
    {
        // for NL80211_IFTYPE_STATION, wsid would be 0, 1, e.
        // for NL80211_IFTYPE_AP, wisd would be 0, 1, f for no WSID WATCH LIST
        // for NL80211_IFTYPE_AP, wisd would be 0~7 for WSID WATCH list_head
        sta = ssv6xxx_find_sta_by_rx_skb(sc, rx_skb);

        // Encrypted RX packet must be for connected STA.
        if (sta == NULL)
            goto drop_rx;
            
        sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
        //tx_desc->tx_report = 1;

        vif = sta_priv->sta_info->vif;
        if (vif == NULL)
            goto drop_rx;

        if (is_broadcast_ether_addr(hdr->addr1))
        {
            vif_priv = (struct ssv_vif_priv_data *)vif->drv_priv;
            rx_hw_dec = vif_priv->has_hw_decrypt;
            do_sw_dec = vif_priv->need_sw_decrypt;
        }
        else
        {
            rx_hw_dec = sta_priv->has_hw_decrypt;
            do_sw_dec = sta_priv->need_sw_decrypt;
        }
        #if 0
        if (rx_count++ < 20)
        {
            printk(KERN_ERR "HW DEC (%d - %d) %d %02X:%02X:%02X:%02X:%02X:%02X\n",
                   rx_hw_dec, do_sw_dec, rxdesc->wsid,
                   hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
                   hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
            _ssv6xxx_hexdump("M ", (const u8 *)rx_skb->data, (rx_skb->len > 128) ? 128 : rx_skb->len);
        }
        #endif // 0
        #if 0
        dev_err(sc->dev, "R %02X:%02X:%02X:%02X:%02X:%02X %d %d\n",
                hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
                hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
                rx_hw_dec, do_sw_dec);
        _ssv6xxx_hexdump("R ", (const u8 *)rx_skb->data,
                         (rx_skb->len > 128) ? 128 : rx_skb->len);
        #endif // 0
    }

    if (sc->dbg_rx_frame)
    {
        _ssv6xxx_hexdump("================================================================\n"
                         "RX frame", (const u8 *)rx_skb->data, rx_skb->len);
    }

    /* remove ssv6xxx headers before passing to mac80211 */
    skb_pull(rx_skb, SSV6XXX_RX_DESC_LEN);
    skb_trim(rx_skb, rx_skb->len-sc->sh->rx_pinfo_pad);




#ifdef CONFIG_P2P_NOA    
    if(ieee80211_is_beacon(hdr->frame_control))
            ssv6xxx_noa_detect(sc, hdr, rx_skb->len);
#endif


    

#ifdef USE_LOCAL_CRYPTO    
    if ((rx_hw_dec == false) && (do_sw_dec == true))
    {
#ifndef MULTI_THREAD_ENCRYPT
        ret = ssv6xxx_skb_decrypt(rx_skb, sta, sc);
        if (ret < 0)
        {
            dev_err(sc->dev, "[Local Crypto]: Fail to decrypt local: %02X:%02X:%02X:%02X:%02X:%02X, ret = %d.\n",
                hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
                hdr->addr2[3], hdr->addr2[4], hdr->addr2[5], ret);
            goto drop_rx;
        }
#else
        skb_info->sta = sta;
        ret = ssv6xxx_skb_pre_decrypt(rx_skb, sta, sc);            
        if (ret == 0)
        {                        
            skb_info->crypt_st = PKT_CRYPT_ST_DEC_PRE;
            spin_lock_irqsave(&sc->crypt_st_lock, flags);
            __skb_queue_tail(&sc->preprocess_q, rx_skb);
            spin_unlock_irqrestore(&sc->crypt_st_lock, flags);

            list_for_each_entry_reverse(ta, &encrypt_task_head, list)
            {
                if ((cpu_online(ta->cpu_no)) && (ta->running == 0))
                {
                    wake_up(&ta->encrypt_wait_q);
                    return;
                }
            }
            return;
        }
        else if (ret ==(-EOPNOTSUPP))
        {
            //printk("[MT-CRYPTO]: Not support MT-Crypto\n");
            ret = ssv6xxx_skb_decrypt(rx_skb, sta, sc);
            if (ret < 0)
            {
                dev_err(sc->dev, "[Local Crypto]: Fail to decrypt local: %02X:%02X:%02X:%02X:%02X:%02X, ret = %d.\n",
                    hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
                    hdr->addr2[3], hdr->addr2[4], hdr->addr2[5], ret);
                goto drop_rx;
            }
        }
        else
        {
            printk("[MT-CRYPTO]: Failed to do pre-decrypt (%d)\n", ret);
            dev_kfree_skb_any(rx_skb);
            return;
        }
#endif //ifndef MULTI_THREAD_ENCRYPT
    }
#endif

    if (rx_hw_dec || do_sw_dec)
    {
        hdr = (struct ieee80211_hdr *)rx_skb->data;
        rxs = IEEE80211_SKB_RXCB(rx_skb);
        hdr->frame_control = hdr->frame_control & ~(cpu_to_le16(IEEE80211_FCTL_PROTECTED));
        rxs->flag |= (RX_FLAG_DECRYPTED|RX_FLAG_IV_STRIPPED);
    }

    sc->rx.num_pkts++;

    // Debug code to check received broadcast frame.
#if 0
    if (   is_broadcast_ether_addr(hdr->addr1)
        && (ieee80211_is_data_qos(fc) || ieee80211_is_data(fc)))
#endif
#if 0
    if (ieee80211_is_probe_req(fc))
    {
        #if 0
        printk(KERN_ERR "RX M: 1 %02X:%02X:%02X:%02X:%02X:%02X  (%d - %d - %d)\n",
               hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
               hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
               (le16_to_cpu(hdr->seq_ctrl) >> 4),
               rxdesc->wsid, ieee80211_has_protected(fc));
        #endif
        //printk(KERN_ERR "RX M: 2 %02X:%02X:%02X:%02X:%02X:%02X\n",
        printk(KERN_ERR "Probe Req: 2 %02X:%02X:%02X:%02X:%02X:%02X\n",
               hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
               hdr->addr2[3], hdr->addr2[4], hdr->addr2[5]);
        #if 0
        printk(KERN_ERR "RX M: 3 %02X:%02X:%02X:%02X:%02X:%02X\n",
               hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
               hdr->addr3[3], hdr->addr3[4], hdr->addr3[5]);
        #endif
        _ssv6xxx_hexdump("RX frame", (const u8 *)rx_skb->data,
                         (rx_skb->len > 128) ? 128 : rx_skb->len);
    }
#endif // DEBUG CODE

    #if defined(USE_THREAD_RX) && !defined(IRQ_PROC_RX_DATA)
    local_bh_disable();
    ieee80211_rx(sc->hw, rx_skb);
    local_bh_enable();
    #else
    ieee80211_rx_irqsafe(sc->hw, rx_skb);
    #endif // USE_THREAD_RX
	return;

drop_rx:
#if 0
    dev_err(sc->dev, "D %02X:%02X:%02X:%02X:%02X:%02X\n",
            hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
            hdr->addr2[3], hdr->addr2[4], hdr->addr2[5]);
#endif
    dev_kfree_skb_any(rx_skb);
} // end of - _proc_data_rx_skb -


#ifdef IRQ_PROC_RX_DATA
static struct sk_buff *_proc_rx_skb (struct ssv_softc *sc, struct sk_buff *rx_skb)
{
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)(rx_skb->data + SSV6XXX_RX_DESC_LEN);
    struct ssv6200_rx_desc *rxdesc = (struct ssv6200_rx_desc *)rx_skb->data;

    // Process BA later
    if (   ieee80211_is_back(hdr->frame_control)
        || (rxdesc->c_type == HOST_EVENT))
        return rx_skb;

    _proc_data_rx_skb(sc, rx_skb);
    return NULL;
} // end of - _proc_rx_skb -
#endif // IRQ_PROC_RX_DATA


void _process_rx_q (struct ssv_softc *sc, struct sk_buff_head *rx_q, spinlock_t *rx_q_lock)
{
    struct sk_buff                *skb;
    struct ieee80211_hdr          *hdr;
    struct ssv6200_rx_desc        *rxdesc;
    unsigned long                  flags=0;

    #ifdef USE_FLUSH_RETRY
    bool has_ba_processed = false;
    #endif // USE_FLUSH_RETRY
    
    while (1) {
        if (rx_q_lock != NULL)
        {
            spin_lock_irqsave(rx_q_lock, flags);
            skb = __skb_dequeue(rx_q);
        }
        else
            skb = skb_dequeue(rx_q);

        if (!skb)
        {
            if (rx_q_lock != NULL)
                spin_unlock_irqrestore(rx_q_lock, flags);
            break;
        }
        sc->rx.rxq_count --;
        if (rx_q_lock != NULL)
            spin_unlock_irqrestore(rx_q_lock, flags);

        /* extract headers */
        rxdesc = (struct ssv6200_rx_desc *)skb->data;

        if (rxdesc->c_type == HOST_EVENT)
        {
            struct cfg_host_event *h_evt = (struct cfg_host_event *)rxdesc;
            if (h_evt->h_event == SOC_EVT_NO_BA)
            {
                ssv6200_ampdu_no_BA_handler(sc->hw, skb);
                #ifdef USE_FLUSH_RETRY
                has_ba_processed = true;
                #endif
                // NO BA event skb will be reused by rate control. Do not free it.
                // dev_kfree_skb_any(skb);
            }
            else if (h_evt->h_event == SOC_EVT_RC_MPDU_REPORT)
            {
                #if 0
                struct cfg_host_event *host_event;
                struct firmware_rate_control_report_data *report_data;
                host_event = (struct cfg_host_event *)rxdesc;
                report_data = (struct firmware_rate_control_report_data *)&host_event->dat[0];
                printk("MPDU report get!!wsid[%d]didx[%d]F[%d]S[%d]\n",report_data->wsid,report_data->rates[0].data_rate,report_data->ampdu_len,report_data->ampdu_ack_len);
                //dev_kfree_skb_any(skb);
                #endif // 0

                skb_queue_tail(&sc->rc_report_queue, skb);
                if (sc->rc_sample_sechedule == 0)
                    queue_work(sc->rc_sample_workqueue, &sc->rc_sample_work);
            }
            else if (h_evt->h_event == SOC_EVT_SDIO_TEST_COMMAND)
            {
                if(h_evt->evt_seq_no == 0)
                {
                    printk("SOC_EVT_SDIO_TEST_COMMAND\n");
                    sc->sdio_rx_evt_size = h_evt->len;
                    sc->sdio_throughput_timestamp = jiffies;
                }
                else
                {
                    sc->sdio_rx_evt_size += h_evt->len;
                    if (time_after(jiffies, sc->sdio_throughput_timestamp + msecs_to_jiffies(1000)))
                    {
                        printk("data[%ld] SDIO RX througjput %ld Kbps\n",sc->sdio_rx_evt_size,(sc->sdio_rx_evt_size << 3) / jiffies_to_msecs(jiffies - sc->sdio_throughput_timestamp));
                        sc->sdio_throughput_timestamp = jiffies;
                        sc->sdio_rx_evt_size = 0;
                    }
                }
                dev_kfree_skb_any(skb);
            }
            else if (h_evt->h_event == SOC_EVT_RESET_HOST)
            {
                dev_kfree_skb_any(skb);
                ssv6xxx_restart_hw(sc);            
#ifdef CONFIG_P2P_NOA
            }else if(h_evt->h_event == SOC_EVT_NOA){

                ssv6xxx_process_noa_event(sc, skb);
                dev_kfree_skb_any(skb);
#endif
            }else{
            ;}

            continue;
        }

        hdr = (struct ieee80211_hdr *)(skb->data + SSV6XXX_RX_DESC_LEN);
        if (ieee80211_is_back(hdr->frame_control))
        {
            ssv6200_ampdu_BA_handler(sc->hw, skb);
            #ifdef USE_FLUSH_RETRY
            has_ba_processed = true;
            #endif // USE_FLUSH_RETRY
            continue;
        }
        
        _proc_data_rx_skb(sc, skb);
    }

    #ifdef USE_FLUSH_RETRY
    if (has_ba_processed)
    {
        ssv6xxx_ampdu_postprocess_BA(sc->hw);
    }
    #endif // USE_FLUSH_RETRY
} // end of - _process_rx_q -

#if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
int ssv6200_rx(struct sk_buff_head *rx_skb_q, void *args)
#else
int ssv6200_rx(struct sk_buff *rx_skb, void *args)
#endif
{
    struct ssv_softc *sc=args;

    #ifdef IRQ_PROC_RX_DATA
    struct sk_buff *skb;
    skb = _proc_rx_skb(sc, rx_skb);
    if (skb == NULL)
        return 0;
    #endif // IRQ_PROC_TX_DATA

    #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
    {
    unsigned long flags;
    spin_lock_irqsave(&sc->rx_skb_q.lock, flags);
    while (skb_queue_len(rx_skb_q))
        __skb_queue_tail(&sc->rx_skb_q, __skb_dequeue(rx_skb_q));
    spin_unlock_irqrestore(&sc->rx_skb_q.lock, flags);
    }
    #else
    skb_queue_tail(&sc->rx_skb_q, rx_skb);
    #endif

    wake_up_interruptible(&sc->rx_wait_q);

    return 0;
}

struct ieee80211_sta *ssv6xxx_find_sta_by_rx_skb (struct ssv_softc *sc, struct sk_buff *skb)
{
    struct ieee80211_hdr     *hdr = (struct ieee80211_hdr *)(skb->data + SSV6XXX_RX_DESC_LEN);
    struct ssv6200_rx_desc   *rxdesc = (struct ssv6200_rx_desc *)skb->data;;

    if ((rxdesc->wsid >= 0) && (rxdesc->wsid < SSV_NUM_STA))
        return sc->sta_info[rxdesc->wsid].sta;
    else
        return ssv6xxx_find_sta_by_addr(sc, hdr->addr2);
}

struct ieee80211_sta *ssv6xxx_find_sta_by_addr (struct ssv_softc *sc, u8 addr[6])
{
    struct ieee80211_sta *sta;
    int i;

    for (i = 0; i < SSV6200_MAX_VIF; i++)
    {
        if (sc->vif_info[i].vif == NULL)
            continue;

        sta = ieee80211_find_sta(sc->vif_info[i].vif, addr);
        if (sta != NULL)
            return sta;
    }
    return NULL;
} // end of - ssv6xxx_find_sta -


void ssv6xxx_foreach_sta (struct ssv_softc *sc, void (*sta_func)(struct ssv_softc *, struct ssv_sta_info *, void *), void *param)
{
    int i;

    BUG_ON(sta_func == NULL);

#if 0
    for (i = 0; i < SSV6200_MAX_VIF; i++)
    {
        struct ssv_vif_priv_data *vif_priv;
        int j;

        if (sc->vif_info[i].vif == NULL)
            continue;

        vif_priv = (struct ssv_vif_priv_data *)sc->vif[i]->drv_priv;
        for (j = 0; j < SSV_NUM_STA; j++)
        {
            if ((vif_priv->sta_info[j].s_flags & STA_FLAG_VALID) == 0)
                continue;
            (*sta_func)(sc, &vif_priv->sta_info[j], param);
        }
    }
#else
    for (i = 0; i < SSV_NUM_STA; i++)
    {
        if ((sc->sta_info[i].s_flags & STA_FLAG_VALID) == 0)
            continue;

        (*sta_func)(sc, &sc->sta_info[i], param);
    }
#endif
}


void ssv6xxx_foreach_vif_sta (struct ssv_softc *sc,
                              struct ssv_vif_info *vif_info,
                              void (*sta_func)(struct ssv_softc *,
                                               struct ssv_vif_info *,
                                               struct ssv_sta_info *,
                                               void *),
                              void *param)
{
    struct ssv_vif_priv_data *vif_priv;
    struct ssv_sta_priv_data *sta_priv_iter;

    BUG_ON(vif_info == NULL);
    BUG_ON((size_t)vif_info < 0x30000);

    vif_priv = (struct ssv_vif_priv_data *)vif_info->vif->drv_priv;
    BUG_ON((size_t)vif_info->vif < 0x30000);
    BUG_ON((size_t)vif_priv < 0x30000);

    list_for_each_entry(sta_priv_iter, &vif_priv->sta_list, list)
    {
        BUG_ON(sta_priv_iter == NULL);
        BUG_ON((size_t)sta_priv_iter < 0x30000);
        BUG_ON(sta_priv_iter->sta_info == NULL);
        BUG_ON((size_t)sta_priv_iter->sta_info   < 0x30000);

        if ((sta_priv_iter->sta_info->s_flags & STA_FLAG_VALID) == 0)
            continue;
        (*sta_func)(sc, vif_info, sta_priv_iter->sta_info, param);
    }
}

