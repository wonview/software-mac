/*
 * Copyright (c) 2014 South Silicon Valley Microelectronics Inc.
 * Copyright (c) 2015 iComm Semiconductor Ltd.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include <linux/irq.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "sdio_def.h"
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/firmware.h>
#ifdef  CONFIG_FW_ALIGNMENT_CHECK
#include <linux/skbuff.h>
#endif

// Freddie ToDo: DO NOT use static data.
static struct ssv6xxx_platform_data wlan_data;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
#include <linux/printk.h>
#else
#include <linux/kernel.h>
#endif

#include <ssv6200.h>

#define MAX_RX_FRAME_SIZE 0x900

#define SSV_VENDOR_ID	0x3030
#define SSV_CABRIO_DEVID	0x3030

#define ENABLE_FW_SELF_CHECK          1
//32K
#define FW_BLOCK_SIZE                   0x8000
#define CHECKSUM_BLOCK_SIZE             1024
#define FW_CHECKSUM_INIT                (0x12345678)

#define FW_STATUS_REG   ADR_TX_SEG
#define FW_STATUS_MASK  (0x00FF0000)

extern void *ssv6xxx_ifdebug_info[];
extern int ssv_devicetype;

struct ssv6xxx_sdio_glue
{
    struct device              *dev;
    struct platform_device     *core;
#ifdef  CONFIG_FW_ALIGNMENT_CHECK
    struct sk_buff *dmaSkb;
#endif
#ifdef CONFIG_PM
    struct sk_buff *cmd_skb;
#endif
    /* for ssv SDIO */
    unsigned int                dataIOPort;
    unsigned int                regIOPort;

    irq_handler_t               irq_handler;
    void                       *irq_dev;
    bool                        dev_ready;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
static const struct sdio_device_id ssv6xxx_sdio_devices[] __devinitconst =
#else
static const struct sdio_device_id ssv6xxx_sdio_devices[] =
#endif
{
    { SDIO_DEVICE(SSV_VENDOR_ID, SSV_CABRIO_DEVID) },
    {}
};
MODULE_DEVICE_TABLE(sdio, ssv6xxx_sdio_devices);

static bool ssv6xxx_is_ready (struct device *child)
{
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
        return false;

    return glue->dev_ready;
} // end of - ssv6xxx_is_ready -

static int ssv6xxx_sdio_cmd52_read(struct device *child, u32 addr,
        u32 *value)
{
    int ret = -1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        *value = sdio_readb(func, addr, &ret);
        sdio_release_host(func);
    }
    return ret;
}


static int ssv6xxx_sdio_cmd52_write(struct device *child, u32 addr,
        u32 value)
{
    int ret = -1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);
        sdio_claim_host(func);
        sdio_writeb(func, value, addr, &ret);
        sdio_release_host(func);
    }
    return ret;
}




static int __must_check ssv6xxx_sdio_read_reg(struct device *child, u32 addr,
        u32 *buf)
{    
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func ;
    u8 data[4];
        
    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;
   
    //dev_err(child->parent, "sdio read reg device[%08x] parent[%08x]\n",child,child->parent);

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);
        sdio_claim_host(func);

        // 4 bytes address
        data[0] = (addr >> ( 0 )) &0xff;
        data[1] = (addr >> ( 8 )) &0xff;
        data[2] = (addr >> ( 16 )) &0xff;
        data[3] = (addr >> ( 24 )) &0xff;

        //8 byte ( 4 bytes address , 4 bytes data )
        ret = sdio_memcpy_toio(func, glue->regIOPort, data, 4);
        if (WARN_ON(ret))
        {
            dev_err(child->parent, "sdio read reg write address failed (%d)\n", ret);
            goto io_err;
        }

        ret = sdio_memcpy_fromio(func, data, glue->regIOPort, 4);
        if (WARN_ON(ret))
        {
            dev_err(child->parent, "sdio read reg from I/O failed (%d)\n",ret);
        	goto io_err;
    	 }

        if(ret == 0)
        {
            *buf = (data[0]&0xff);
            *buf = *buf | ((data[1]&0xff)<<( 8 ));
            *buf = *buf | ((data[2]&0xff)<<( 16 ));
            *buf = *buf | ((data[3]&0xff)<<( 24 ));
        }
        else
            *buf = 0xffffffff;
io_err:
        sdio_release_host(func);
        //dev_dbg(child->parent, "sdio read reg addr 0x%x, 0x%x  ret:%d\n", addr, *buf,ret);
    }
    else
    {
        dev_err(child->parent, "sdio read reg glue == NULL!!!\n");
    }

    //if (WARN_ON(ret))
    //  dev_err(child->parent, "sdio read reg failed (%d)\n", ret);

    return ret;
}

#ifdef ENABLE_WAKE_IO_ISR_WHEN_HCI_ENQUEUE
static int ssv6xxx_sdio_trigger_tx_rx (struct device *child)
{
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    struct mmc_host *host;

    if (glue == NULL)
        return ret;

    func = dev_to_sdio_func(glue->dev);
    host = func->card->host;

    host->ops->enable_sdio_irq(host, 0);
    wake_up_process(host->sdio_irq_thread);

    return 0;
}
#endif // ENABLE_WAKE_IO_ISR_WHEN_HCI_ENQUEUE

static int __must_check ssv6xxx_sdio_write_reg(struct device *child, u32 addr,
        u32 buf)
{
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    u8 data[8];

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;
    
    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);
        dev_dbg(child->parent, "sdio write reg addr 0x%x, 0x%x\n",addr, buf);

        sdio_claim_host(func);

        // 4 bytes address
        data[0] = (addr >> ( 0 )) &0xff;
        data[1] = (addr >> ( 8 )) &0xff;
        data[2] = (addr >> ( 16 )) &0xff;
        data[3] = (addr >> ( 24 )) &0xff;

        // 4 bytes data
        data[4] = (buf >> ( 0 )) &0xff;
        data[5] = (buf >> ( 8 )) &0xff;
        data[6] = (buf >> ( 16 )) &0xff;
        data[7] = (buf >> ( 24 )) &0xff;

        //8 byte ( 4 bytes address , 4 bytes data )
        ret = sdio_memcpy_toio(func, glue->regIOPort, data, 8);

        sdio_release_host(func);

        //if (WARN_ON(ret))
        //  dev_err(child->parent, "sdio write reg failed (%d)\n", ret);
    }
    else
    {
        dev_err(child->parent, "sdio write reg glue == NULL!!!\n");
    }
    return ret;
}

// Write to SSV6XXX's SRAM 
static int ssv6xxx_sdio_write_sram(struct device *child, u32 addr, u8 *data, u32 size)
{
    int     ret = -1;
    struct ssv6xxx_sdio_glue *glue;
    struct sdio_func *func=NULL;
    
    glue = dev_get_drvdata(child->parent);

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
        return ret;

    func = dev_to_sdio_func(glue->dev);
    sdio_claim_host(func);
    do {
        //Setting SDIO DMA address
        if (ssv6xxx_sdio_write_reg(child,0xc0000860,addr)) ;
    
        // Set data path to DMA to SRAM
        sdio_writeb(func, 0x2, REG_Fn1_STATUS, &ret);
        if (unlikely(ret)) break;

        ret = sdio_memcpy_toio(func, glue->dataIOPort, data, size);
        if (unlikely(ret)) return ret;
    
            // Set data path back to packet
        sdio_writeb(func, 0, REG_Fn1_STATUS, &ret);
        if (unlikely(ret)) return ret;
    } while (0);
    
    sdio_release_host(func);

    return ret;
}

#ifdef CONFIG_SSV_OPENFILE_LOADFW

void * ssv6xxx_open_firmware(char *user_mainfw)
{
    struct file *fp;


    fp = filp_open(user_mainfw, O_RDONLY, 0);
    if (IS_ERR(fp))
    {
        printk("ssv6xxx_open_firmware failed!!\n");
        fp = NULL;
    }

    return fp;
}
int ssv6xxx_read_fw_block(char *buf, int len, void *image)
{
	struct file *fp = (struct file *)image;
	int rdlen;

	if (!image)
		return 0;

	rdlen = kernel_read(fp, fp->f_pos, buf, len);
	if (rdlen > 0)
		fp->f_pos += rdlen;

	return rdlen;
}


void ssv6xxx_close_firmware(void *image)
{

	if (image)
		filp_close((struct file *)image, NULL);
}


static int ssv6xxx_sdio_load_firmware(struct device *child)
{
    int ret = 0;
    struct ssv6xxx_sdio_glue *glue;
    u8   *fw_buffer = NULL;
    u32   sram_addr = 0x00000000;
    u32   block_count = 0;
    u32   res_size=0,len=0,tolen=0;
    void *fw_fp=NULL;
#ifdef ENABLE_FW_SELF_CHECK
    u32   checksum = FW_CHECKSUM_INIT;
    u32   fw_checksum,fw_clkcnt;
    u32   retry_count = 3;
    u32  *fw_data32;
#else
    int   writesize=0;
    u32   retry_count = 1;
#endif
    u32   word_count,i;
    u32   j,jblk;
    struct sdio_func *func=NULL;
#if SSV_XTAL==24
    struct mmc_card *card = NULL;
    struct mmc_host *host = NULL;
#endif

    glue = dev_get_drvdata(child->parent);

    if (   (wlan_data.is_enabled != false)
        || (glue != NULL)
        || (glue->dev_ready != false))
    {
        func = dev_to_sdio_func(glue->dev);
#if SSV_XTAL==24
        card = func->card;
        host = card->host;
#endif
        // Load firmware
        fw_fp = ssv6xxx_open_firmware(EXTERNAL_SSV6XXX_FW_NAME);
        if (!fw_fp) {
            printk("failed to find firmware (%s)\n", EXTERNAL_SSV6XXX_FW_NAME);
            ret = -1;
            goto out;
        }

        // Allocate buffer firmware aligned with FW_BLOCK_SIZE and padding with 0xA5 in empty space.
        fw_buffer = (u8 *)kzalloc(FW_BLOCK_SIZE, GFP_KERNEL);
        if (fw_buffer == NULL) {
            printk("Failed to allocate buffer for firmware.\n");
            goto out;
        }


        do {
            u32 clk_en;
            // Reset CPU
            if(1){
            if(ssv6xxx_sdio_write_reg(child, ADR_BRG_SW_RST, 0x0));
            if(ssv6xxx_sdio_write_reg(child, ADR_BOOT, 0x01));

            if(ssv6xxx_sdio_read_reg(child, ADR_PLATFORM_CLOCK_ENABLE, &clk_en));
            if(ssv6xxx_sdio_write_reg(child, ADR_PLATFORM_CLOCK_ENABLE, clk_en | (1 << 2)));
            }
            // Write firmware to SRAM address 0

            printk("Writing firmware to SSV6XXX...\n");
            
            memset(fw_buffer, 0xA5, FW_BLOCK_SIZE);            
            while ((len = ssv6xxx_read_fw_block((char*)fw_buffer, FW_BLOCK_SIZE, fw_fp))) {
                tolen += len;
                //printk("read len=%d,sram_addr=%d\n",len,sram_addr);
                if(len < FW_BLOCK_SIZE){
                    res_size = len;
                    break;
                }
                if(0) //byte mode
                {
                    jblk = len / 128;
                    for(j=0;j<jblk;j++)
                    {
                        ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)(fw_buffer+j*128), 128);
                        if (ret){
                            printk("ssv6xxx_sdio_write_sram failed!!\n");
                            break;
                        }
                        sram_addr += 128;
                    }
                }else{
                    ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, FW_BLOCK_SIZE);
                    if (ret)
                        break;
                    sram_addr += FW_BLOCK_SIZE;
                }
                                
                word_count = (len / sizeof(u32));
                fw_data32 = (u32 *)fw_buffer;
                for (i = 0; i < word_count; i++){
                    checksum += fw_data32[i];
                }
                //printk("\nper blk cks=%x\n",checksum);                
                memset(fw_buffer, 0xA5, FW_BLOCK_SIZE);          
            }
            
            if(res_size)
            {
                u32 cks_blk_cnt,cks_blk_res;
                cks_blk_cnt = res_size / CHECKSUM_BLOCK_SIZE;                    
                cks_blk_res = res_size % CHECKSUM_BLOCK_SIZE;
                //printk("res_size = %d,cks_blk_cnt=%d\n",res_size,cks_blk_cnt);
                if(0) //byte mode
                {
                    jblk = ((cks_blk_cnt+1)*CHECKSUM_BLOCK_SIZE)/128;
                    for(j=0;j<jblk;j++){
                        ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)(fw_buffer+j*128), 128);
                        sram_addr += 128;
                    }
                }else{
                    ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, (cks_blk_cnt+1)*CHECKSUM_BLOCK_SIZE);
                }
                
                word_count = (cks_blk_cnt * CHECKSUM_BLOCK_SIZE / sizeof(u32));
                fw_data32 = (u32 *)fw_buffer;
                for (i = 0; i < word_count; i++)
                    checksum += *fw_data32++;
                //printk("last cks=%x,tolen=%d,res_size=%d\n",checksum,tolen,res_size);
                if(cks_blk_res)
                {
                    word_count = (CHECKSUM_BLOCK_SIZE / sizeof(u32));
                    //fw_data32 = (u32 *)&fw_buffer[cks_blk_cnt * CHECKSUM_BLOCK_SIZE];
                    for (i = 0; i < word_count; i++) {
                        checksum += *fw_data32++;
                    }
                }
                //printk("over cks=%x\n",checksum);
            }
            
            // Calculate the final checksum.
            checksum = ((checksum >> 24) + (checksum >> 16) + (checksum >> 8) + checksum) & 0x0FF;
            checksum <<= 16;

            if (ret == 0) {

                block_count = tolen / CHECKSUM_BLOCK_SIZE;
                res_size = tolen % CHECKSUM_BLOCK_SIZE;
                if(res_size)
                    block_count++;
                // Inform FW that how many blocks is downloaded such that FW can calculate the checksum.
                if(ssv6xxx_sdio_write_reg(child, FW_STATUS_REG, (block_count << 16)));

                if(ssv6xxx_sdio_read_reg(child, FW_STATUS_REG, &fw_clkcnt));
                printk("(block_count << 16) = %x,reg =%x\n",(block_count << 16),fw_clkcnt);
                // Release reset to let CPU run.
                if(ssv6xxx_sdio_write_reg(child, ADR_BRG_SW_RST, 0x1));
                printk("Firmware \"%s\" loaded\n", EXTERNAL_SSV6XXX_FW_NAME);
                // Wait FW to calculate checksum.
                msleep(50);
                // Check checksum result and set to complement value if checksum is OK.
                if(ssv6xxx_sdio_read_reg(child, FW_STATUS_REG, &fw_checksum));
                fw_checksum = fw_checksum & FW_STATUS_MASK;
                if (fw_checksum == checksum) {
                    if(ssv6xxx_sdio_write_reg(child, FW_STATUS_REG, (~checksum & FW_STATUS_MASK)));
                    ret = 0;
                    printk("Firmware check OK.%04x = %04x\n", fw_checksum, checksum);
                    break;
                } else {
                    printk("FW checksum error: %04x != %04x\n", fw_checksum, checksum);
                    ret = -1;
                }

            } else {
                printk("Firmware \"%s\" download failed. (%d)\n", EXTERNAL_SSV6XXX_FW_NAME, ret);
                ret = -1;
            }
        } while (--retry_count);

        if (ret)
            goto out;

        ret = 0;
    }

out:
    if(fw_fp)
        ssv6xxx_close_firmware(fw_fp);
    
    if (fw_buffer != NULL)
        kfree(fw_buffer);

    // Sleep to let SSV6XXX get ready.
    msleep(50);

#if SSV_XTAL==24
    if(host)
    {
        //Setting SDIO to 50M
        host->ios.clock = 50000000;
        host->ops->set_ios(host,&host->ios);
        mdelay(20);
    }
#endif // SSV_XTAL

    return ret;
}

#else // CONFIG_SSV_OPENFILE_LOADFW

static char ssv6xxx_fw_name[50] = "ssv6051-sw.bin";

int ssv6xxx_get_firmware(struct device *dev,
            char *user_mainfw,
            const struct firmware **mainfw)
{
    int ret;

    //hBUG_ON(helper == NULL);
    BUG_ON(mainfw == NULL);

    /* Try user-specified firmware first */
    if (*user_mainfw) {
        ret = request_firmware(mainfw, user_mainfw, dev);
        if (ret) {
            dev_err(dev, "couldn't find main firmware %s\n",user_mainfw);
            goto fail;
        }
        if (*mainfw)
            return 0;
    }

fail:
    /* Failed */
    if (*mainfw) {
        release_firmware(*mainfw);
        *mainfw = NULL;
    }

    return -ENOENT;
}

static int ssv6xxx_sdio_load_firmware(struct device *child)
{
    int ret = 0;
    const struct firmware *ssv6xxx_fw = NULL;
    struct ssv6xxx_sdio_glue *glue;
    u8   *fw_buffer = NULL;
    u32   sram_addr = 0x00000000;
    u32   block_count = 0;
    u32   block_idx = 0;
    u32   res_size;
    u8   *fw_data;
#ifdef ENABLE_FW_SELF_CHECK
    u32   checksum = FW_CHECKSUM_INIT;
    u32   fw_checksum;
    u32   retry_count = 3;
    u32  *fw_data32;
#else
    int   writesize=0;
    u32   retry_count = 1;
#endif

    struct sdio_func *func=NULL;
#if SSV_XTAL==24
    struct mmc_card *card = NULL;
    struct mmc_host *host = NULL;
#endif

    glue = dev_get_drvdata(child->parent);
    if (   (wlan_data.is_enabled != false)
        || (glue != NULL)
        || (glue->dev_ready != false))
    {
        func = dev_to_sdio_func(glue->dev);
#if SSV_XTAL==24
        card = func->card;
        host = card->host;
#endif

        // Load firmware
        ret = ssv6xxx_get_firmware(glue->dev, ssv6xxx_fw_name, &ssv6xxx_fw);
        if (ret) {
            pr_err("failed to find firmware (%d)\n", ret);
            goto out;
        }

        // Allocate buffer firmware aligned with FW_BLOCK_SIZE and padding with 0xA5 in empty space.
        fw_buffer = (u8 *)kzalloc(FW_BLOCK_SIZE, GFP_KERNEL);
        if (fw_buffer == NULL) {
            pr_err("Failed to allocate buffer for firmware.\n");
            goto out;
        }

#ifdef ENABLE_FW_SELF_CHECK
        block_count = ssv6xxx_fw->size / CHECKSUM_BLOCK_SIZE;
        res_size = ssv6xxx_fw->size % CHECKSUM_BLOCK_SIZE;
        {
            int word_count = (int)(block_count * CHECKSUM_BLOCK_SIZE / sizeof(u32));
            int i;
            fw_data32 = (u32 *)ssv6xxx_fw->data;
            for (i = 0; i < word_count; i++)
                checksum += fw_data32[i];

            if(res_size)
            {
                memset(fw_buffer, 0xA5, CHECKSUM_BLOCK_SIZE);
                memcpy(fw_buffer, &ssv6xxx_fw->data[block_count * CHECKSUM_BLOCK_SIZE], res_size);

                // Accumulate checksum for the incomplete block
                word_count = (int)(CHECKSUM_BLOCK_SIZE / sizeof(u32));
                fw_data32 = (u32 *)fw_buffer;
                for (i = 0; i < word_count; i++) {
                    checksum += fw_data32[i];
                }
            }
        }

        // Calculate the final checksum.
        checksum = ((checksum >> 24) + (checksum >> 16) + (checksum >> 8) + checksum) & 0x0FF;
        checksum <<= 16;
#endif // ENABLE_FW_SELF_CHECK

        do {
            u32 clk_en;
            // Reset CPU
            if(ssv6xxx_sdio_write_reg(child, ADR_BRG_SW_RST, 0x0));
            if(ssv6xxx_sdio_write_reg(child, ADR_BOOT, 0x01));

            if(ssv6xxx_sdio_read_reg(child, ADR_PLATFORM_CLOCK_ENABLE, &clk_en));
            if(ssv6xxx_sdio_write_reg(child, ADR_PLATFORM_CLOCK_ENABLE, clk_en | (1 << 2)));

            // Write firmware to SRAM address 0
#ifdef ENABLE_FW_SELF_CHECK
            block_count = ssv6xxx_fw->size / FW_BLOCK_SIZE;
            res_size = ssv6xxx_fw->size % FW_BLOCK_SIZE;

            printk("Writing %d blocks to SSV6XXX...", block_count);
            for (block_idx = 0, fw_data = (u8 *)ssv6xxx_fw->data, sram_addr = 0;block_idx < block_count;
                block_idx++, fw_data += FW_BLOCK_SIZE, sram_addr += FW_BLOCK_SIZE) {
                memcpy(fw_buffer, fw_data, FW_BLOCK_SIZE);
                ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, FW_BLOCK_SIZE);
                if (ret)
                    break;
            }
            if(res_size)
            {
                memset(fw_buffer, 0xA5, FW_BLOCK_SIZE);
                memcpy(fw_buffer, &ssv6xxx_fw->data[block_count * FW_BLOCK_SIZE], res_size);
                ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, ((res_size/CHECKSUM_BLOCK_SIZE)+1)*CHECKSUM_BLOCK_SIZE);
            }
#else // ENABLE_FW_SELF_CHECK
            block_count = ssv6xxx_fw->size / FW_BLOCK_SIZE;
            res_size = ssv6xxx_fw->size % FW_BLOCK_SIZE;
            writesize = sdio_align_size(func,res_size);

            printk("Writing %d blocks to SSV6XXX...", block_count);
            for (block_idx = 0, fw_data = (u8 *)ssv6xxx_fw->data, sram_addr = 0;block_idx < block_count;
                block_idx++, fw_data += FW_BLOCK_SIZE, sram_addr += FW_BLOCK_SIZE) {
                memcpy(fw_buffer, fw_data, FW_BLOCK_SIZE);
                ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, FW_BLOCK_SIZE);
                if (ret)
                    break;
            }
            if(res_size)
            {
                //memset(fw_buffer, 0xA5, FW_BLOCK_SIZE);
                memcpy(fw_buffer, &ssv6xxx_fw->data[block_count * FW_BLOCK_SIZE], res_size);
                ret = ssv6xxx_sdio_write_sram(child, sram_addr, (u8 *)fw_buffer, writesize);
            }
#endif // ENABLE_FW_SELF_CHECK

            if (ret == 0) {
#ifdef ENABLE_FW_SELF_CHECK
                block_count = ssv6xxx_fw->size / CHECKSUM_BLOCK_SIZE;
                res_size = ssv6xxx_fw->size % CHECKSUM_BLOCK_SIZE;
                if(res_size)
                    block_count++;
                // Inform FW that how many blocks is downloaded such that FW can calculate the checksum.
                if(ssv6xxx_sdio_write_reg(child, FW_STATUS_REG, (block_count << 16)));
#endif // ENABLE_FW_SELF_CHECK

                // Release reset to let CPU run.
                if(ssv6xxx_sdio_write_reg(child, ADR_BRG_SW_RST, 0x1));
                printk("Firmware \"%s\" loaded\n", ssv6xxx_fw_name);
#ifdef ENABLE_FW_SELF_CHECK
                // Wait FW to calculate checksum.
                msleep(50);
                // Check checksum result and set to complement value if checksum is OK.
                if(ssv6xxx_sdio_read_reg(child, FW_STATUS_REG, &fw_checksum));
                fw_checksum = fw_checksum & FW_STATUS_MASK;
                if (fw_checksum == checksum) {
                    if(ssv6xxx_sdio_write_reg(child, FW_STATUS_REG, (~checksum & FW_STATUS_MASK)));
                    ret = 0;
                    printk("Firmware check OK.\n");
                    break;
                } else {
                    printk("FW checksum error: %04x != %04x\n", fw_checksum, checksum);
                    ret = -1;
                }
#endif

            } else {
                printk("Firmware \"%s\" download failed. (%d)\n", ssv6xxx_fw_name, ret);
                ret = -1;
            }
        } while (--retry_count);

        if (ret)
            goto out;

        ret = 0;
    }

out:
    if (ssv6xxx_fw)
        release_firmware(ssv6xxx_fw);

    if (fw_buffer != NULL)
        kfree(fw_buffer);

    // Sleep to let SSV6XXX get ready.
    msleep(50);
#if SSV_XTAL==24
    if(host)
    {
        //Setting SDIO to 50M
        host->ios.clock = 50000000;
        host->ops->set_ios(host,&host->ios);
        mdelay(20);
    }
#endif

    return ret;
}

#endif // CONFIG_SSV_OPENFILE_LOADFW

static int ssv6xxx_sdio_irq_getstatus(struct device *child,int *status)
{
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue;

    struct sdio_func *func;
    glue = dev_get_drvdata(child->parent);

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        *status = sdio_readb(func, REG_INT_STATUS, &ret);
        sdio_release_host(func);
    }
    return ret;
}

#if 0
static void _sdio_hexdump(const u8 *buf,
                             size_t len)
{
    size_t i;
    printk("\n-----------------------------\n");
    printk("hexdump(len=%lu):\n", (unsigned long) len);
    {
        for (i = 0; i < len; i++){
            
            printk(" %02x", buf[i]);
            if((i+1)%40 ==0)
                printk("\n");
        }
    }
    printk("\n-----------------------------\n");
}
#endif

static int __must_check ssv6xxx_sdio_read(struct device *child,
        void *buf, size_t *size)
{
    int ret=-1,readsize=0;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func ;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        *size = (uint)sdio_readb(func, REG_CARD_PKT_LEN_0, &ret);
        if (ret)
            dev_err(child->parent, "sdio read hight len failed ret[%d]\n",ret);

        if (ret == 0)
        {
            *size = *size | ((uint)sdio_readb(func, REG_CARD_PKT_LEN_1, &ret)<<0x8);
            if (ret)
                dev_err(child->parent, "sdio read low len failed ret[%d]\n",ret);
        }

        //BUG_ON(*size > 0xffff);
        
        if (ret == 0)
        {
            readsize = sdio_align_size(func,*size);
            ret = sdio_memcpy_fromio(func, buf, glue->dataIOPort, readsize);
            if (ret)
                dev_err(child->parent, "sdio read failed size ret[%d]\n",ret);
        }
        sdio_release_host(func);
    }

#if 0
    if(*size > 1500)
        _sdio_hexdump(buf,*size);
#endif

    return ret;
}

static int __must_check ssv6xxx_sdio_write(struct device *child,
        void *buf, size_t len,u8 queue_num)
{
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    int writesize;
    void *tempPointer;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {

#ifdef  CONFIG_FW_ALIGNMENT_CHECK
        if(((u32)buf)&3)
        {
            //memset(glue->dmaSkb,0x00,2048);
            memcpy(glue->dmaSkb->data,buf,len);
            tempPointer = glue->dmaSkb->data;
        }
        else
#endif
            tempPointer = buf;

#if 0
        if(len > 1500)
            _sdio_hexdump(buf,len);
#endif

        func = dev_to_sdio_func(glue->dev);
        sdio_claim_host(func);
        writesize = sdio_align_size(func,len);

        do
        {
            ret = sdio_memcpy_toio(func, glue->dataIOPort, tempPointer, writesize);
            if ( ret == -EILSEQ || ret == -ETIMEDOUT )
            {
                ret = -1;
                break;
            }
            else
            {
                if(ret)
                    dev_err(glue->dev,"Unexpected return value ret=[%d]\n",ret);
            }
        }
        while( ret == -EILSEQ || ret == -ETIMEDOUT);
        sdio_release_host(func);

        if (ret)
            dev_err(glue->dev, "sdio write failed (%d)\n", ret);
    }
    return ret;
}

static void ssv6xxx_sdio_irq_handler(struct sdio_func *func)
{
    int status;
    struct ssv6xxx_sdio_glue *glue = sdio_get_drvdata(func);
    struct ssv6xxx_platform_data *pwlan_data = &wlan_data;

    //dev_err(&func->dev, "ssv6xxx_sdio_irq_handler [%p] [%p]\n",func,glue);
    //WARN_ON(glue == NULL);

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
        return;

    if ( glue != NULL && glue->irq_handler != NULL )
    {        
        atomic_set(&pwlan_data->irq_handling, 1);
        sdio_release_host(func);
        if ( glue->irq_handler != NULL )
            status = glue->irq_handler(0,glue->irq_dev);
        sdio_claim_host(func);
        atomic_set(&pwlan_data->irq_handling, 0);
    }
}

static void ssv6xxx_sdio_irq_setmask(struct device *child,int mask)
{
    int err_ret;    
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    
    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        sdio_writeb(func,mask, REG_INT_MASK, &err_ret);
        sdio_release_host(func);
    }
}

static void ssv6xxx_sdio_irq_trigger(struct device *child)
{
    int err_ret;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        sdio_writeb(func,0x2, REG_INT_TRIGGER, &err_ret);
        sdio_release_host(func);
    }
}

static int ssv6xxx_sdio_irq_getmask(struct device *child, u32 *mask)
{
    u8 imask = 0;
    int ret=-1;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return ret;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        imask = sdio_readb(func,REG_INT_MASK, &ret);
        *mask = imask;
        sdio_release_host(func);
    }
    return ret;
}


static void ssv6xxx_sdio_irq_enable(struct device *child)
{
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    int ret;
    struct ssv6xxx_platform_data *pwlan_data = &wlan_data;

    if (   (pwlan_data->is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);

//printk("%s(): enable\n", __FUNCTION__);
        //dev_err(child->parent, "ssv6xxx_sdio_irq_enable\n");

        /* Register the isr */
        ret =  sdio_claim_irq(func, ssv6xxx_sdio_irq_handler);
        if (ret)
            dev_err(child->parent, "Failed to claim sdio irq: %d\n", ret);

        sdio_release_host(func);
    }
}

static void ssv6xxx_sdio_irq_disable(struct device *child,bool iswaitirq)
{
    struct ssv6xxx_sdio_glue *glue = NULL;
    struct sdio_func *func;
    struct ssv6xxx_platform_data *pwlan_data = &wlan_data;
    int ret;

    //WARN_ON(true);
    printk("ssv6xxx_sdio_irq_disable\n");
    
    
    if (   (wlan_data.is_enabled == false)
        || (child->parent == NULL))
		return;

    glue = dev_get_drvdata(child->parent);

    if (   (glue == NULL) 
        || (glue->dev_ready == false)
        || (glue->dev == NULL))
        return;

    //if ( glue != NULL && glue->dev!= NULL)
    {
        func = dev_to_sdio_func(glue->dev);
        if(func == NULL){
            printk("func == NULL\n");
            return;
        }
        
        sdio_claim_host(func);
        
        while(atomic_read(&pwlan_data->irq_handling)){
            sdio_release_host(func);
		    schedule_timeout(HZ / 10);
		    sdio_claim_host(func);
        }

        ret = sdio_release_irq(func);
        if (ret)
            dev_err(child->parent, "Failed to release sdio irq: %d\n", ret);

        sdio_release_host(func);

        //ssv6xxx_sdio_irq_setmask(child,mask);
    }
}

static void ssv6xxx_sdio_irq_request(struct device *child,irq_handler_t irq_handler,void *irq_dev)
{
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    bool isIrqEn = false;

    if (   (wlan_data.is_enabled == false)
        || (glue == NULL)
        || (glue->dev_ready == false))
		return;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        //printk("%s(): \n", __FUNCTION__);
        glue->irq_handler = irq_handler;
        glue->irq_dev = irq_dev;

        if (isIrqEn )
        {
            ssv6xxx_sdio_irq_enable(child);
        }
    }
}

static void ssv6xxx_sdio_read_parameter(struct sdio_func *func,
        struct ssv6xxx_sdio_glue *glue)
{
    int err_ret;
    sdio_claim_host(func);

    //get dataIOPort(Accesee packet buffer & SRAM)
    glue->dataIOPort = 0;
    glue->dataIOPort = glue->dataIOPort | (sdio_readb(func, REG_DATA_IO_PORT_0, &err_ret) << ( 8*0 ));
    glue->dataIOPort = glue->dataIOPort | (sdio_readb(func, REG_DATA_IO_PORT_1, &err_ret) << ( 8*1 ));
    glue->dataIOPort = glue->dataIOPort | (sdio_readb(func, REG_DATA_IO_PORT_2, &err_ret) << ( 8*2 ));

    //get regIOPort(Access register)
    glue->regIOPort = 0;
    glue->regIOPort = glue->regIOPort | (sdio_readb(func, REG_REG_IO_PORT_0, &err_ret) << ( 8*0 ));
    glue->regIOPort = glue->regIOPort | (sdio_readb(func, REG_REG_IO_PORT_1, &err_ret) << ( 8*1 ));
    glue->regIOPort = glue->regIOPort | (sdio_readb(func, REG_REG_IO_PORT_2, &err_ret) << ( 8*2 ));

    dev_err(&func->dev, "dataIOPort 0x%x regIOPort 0x%x\n",glue->dataIOPort,glue->regIOPort);

#ifdef CONFIG_PLATFORM_SDIO_BLOCK_SIZE
    err_ret = sdio_set_block_size(func,CONFIG_PLATFORM_SDIO_BLOCK_SIZE);
#else
    err_ret = sdio_set_block_size(func,SDIO_DEF_BLOCK_SIZE);
#endif
    if (err_ret != 0) {
        printk("SDIO setting SDIO_DEF_BLOCK_SIZE fail!!\n");
    }

    // output timing
#ifdef CONFIG_PLATFORM_SDIO_OUTPUT_TIMING
    sdio_writeb(func, CONFIG_PLATFORM_SDIO_OUTPUT_TIMING,REG_OUTPUT_TIMING_REG, &err_ret);
#else
    sdio_writeb(func, SDIO_DEF_OUTPUT_TIMING,REG_OUTPUT_TIMING_REG, &err_ret);
#endif

    // switch to normal mode
    // bit[1] , 0:normal mode, 1: Download mode(For firmware download & SRAM access)
    sdio_writeb(func, 0x00,REG_Fn1_STATUS, &err_ret);

#if 0
    //to check if support tx alloc mechanism
    sdio_writeb(func,SDIO_TX_ALLOC_SIZE_SHIFT|SDIO_TX_ALLOC_ENABLE,REG_SDIO_TX_ALLOC_SHIFT, &err_ret);
#endif

    sdio_release_host(func);
}

static void ssv6xxx_sdio_pmu_wakeup(struct device *child)
{
    int err_ret;
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;

    if ( glue != NULL )
    {
        func = dev_to_sdio_func(glue->dev);

        sdio_claim_host(func);
        //sdio_writeb(func,0x88, REG_PMU_WAKEUP, &err_ret); //FPGA
        sdio_writeb(func,0x01, REG_PMU_WAKEUP, &err_ret);
        mdelay(3);
        sdio_writeb(func,0x00, REG_PMU_WAKEUP, &err_ret);
        sdio_release_host(func);
    }
}












static bool ssv6xxx_sdio_support_scatter(struct device *child)
{
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    bool support = false;
        
    do{

        if(!glue){
            dev_err(child->parent, "ssv6xxx_sdio_enable_scatter glue == NULL!!!\n");
            break;
        }
        
        func = dev_to_sdio_func(glue->dev);                 
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
        if (func->card->host->max_segs < MAX_SCATTER_ENTRIES_PER_REQ) {
            dev_err(child->parent, "host controller only supports scatter of :%d entries, driver need: %d\n",
			func->card->host->max_segs,
			MAX_SCATTER_ENTRIES_PER_REQ);
            break;
	    }
        
        support = true;
#endif
    }while(0);

    return support;
}

static void ssv6xxx_sdio_setup_scat_data(struct sdio_scatter_req *scat_req,
					struct mmc_data *data)
{
	struct scatterlist *sg;
	int i;
    

	data->blksz = SDIO_DEF_BLOCK_SIZE;
	data->blocks = scat_req->len / SDIO_DEF_BLOCK_SIZE;

	printk("scatter: (%s)  (block len: %d, block count: %d) , (tot:%d,sg:%d)\n",
		   (scat_req->req & SDIO_WRITE) ? "WR" : "RD", 
		   data->blksz, data->blocks, scat_req->len,
		   scat_req->scat_entries);

	data->flags = (scat_req->req & SDIO_WRITE) ? MMC_DATA_WRITE :
						    MMC_DATA_READ;

	/* fill SG entries */
	sg = scat_req->sgentries;
	sg_init_table(sg, scat_req->scat_entries);

	/* assemble SG list */
	for (i = 0; i < scat_req->scat_entries; i++, sg++) {
		printk("%d: addr:0x%p, len:%d\n",
			   i, scat_req->scat_list[i].buf,
			   scat_req->scat_list[i].len);

		sg_set_buf(sg, scat_req->scat_list[i].buf,
			   scat_req->scat_list[i].len);
	}

	/* set scatter-gather table for request */
	data->sg = scat_req->sgentries;
	data->sg_len = scat_req->scat_entries;
}

static inline void ssv6xxx_sdio_set_cmd53_arg(u32 *arg, u8 rw, u8 func,
					     u8 mode, u8 opcode, u32 addr,
					     u16 blksz)
{
	*arg = (((rw & 1) << 31) |
		((func & 0x7) << 28) |
		((mode & 1) << 27) |
		((opcode & 1) << 26) |
		((addr & 0x1FFFF) << 9) |
		(blksz & 0x1FF));
}


static int ssv6xxx_sdio_rw_scatter(struct device *child,
			       struct sdio_scatter_req *scat_req)
{   
    struct ssv6xxx_sdio_glue *glue = dev_get_drvdata(child->parent);
    struct sdio_func *func;
    
	struct mmc_request mmc_req;
	struct mmc_command cmd;
	struct mmc_data data;
    
	
	u8 opcode, rw;
	int status = 1;

	

    do{

        if(!glue){
            dev_err(child->parent, "ssv6xxx_sdio_enable_scatter glue == NULL!!!\n");
            break;
        }
        
        func = dev_to_sdio_func(glue->dev);

//Scatter

    	memset(&mmc_req, 0, sizeof(struct mmc_request));
    	memset(&cmd, 0, sizeof(struct mmc_command));
    	memset(&data, 0, sizeof(struct mmc_data));

    	ssv6xxx_sdio_setup_scat_data(scat_req, &data);

    	opcode = 0;//FIXED ADDRESS;

    	rw = (scat_req->req & SDIO_WRITE) ? CMD53_ARG_WRITE : CMD53_ARG_READ;


    	/* set command argument */
    	ssv6xxx_sdio_set_cmd53_arg(&cmd.arg, rw, func->num,
    				  CMD53_ARG_BLOCK_BASIS, opcode, glue->dataIOPort,
    				  data.blocks);

    	cmd.opcode = SD_IO_RW_EXTENDED;
    	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

    	mmc_req.cmd = &cmd;
    	mmc_req.data = &data;

    	mmc_set_data_timeout(&data, func->card);
    	/* synchronous call to process request */
    	mmc_wait_for_req(func->card->host, &mmc_req);

    	status = cmd.error ? cmd.error : data.error;


        if (cmd.error)
    		return cmd.error;
    	if (data.error)
    		return data.error;


    }while(0);


	return status;
}




static struct ssv6xxx_hwif_ops sdio_ops =
{
    .read            = ssv6xxx_sdio_read,
    .write           = ssv6xxx_sdio_write,
    .readreg	        = ssv6xxx_sdio_read_reg,
    .writereg        = ssv6xxx_sdio_write_reg,
#ifdef ENABLE_WAKE_IO_ISR_WHEN_HCI_ENQUEUE
    .trigger_tx_rx   = ssv6xxx_sdio_trigger_tx_rx,
#endif // ENABLE_WAKE_IO_ISR_WHEN_HCI_ENQUEUE
    .irq_getmask     = ssv6xxx_sdio_irq_getmask,
    .irq_setmask     = ssv6xxx_sdio_irq_setmask,
    .irq_enable      = ssv6xxx_sdio_irq_enable,
    .irq_disable     = ssv6xxx_sdio_irq_disable,
    .irq_getstatus   = ssv6xxx_sdio_irq_getstatus,
    .irq_request     = ssv6xxx_sdio_irq_request,
    .irq_trigger     = ssv6xxx_sdio_irq_trigger,
    .pmu_wakeup      = ssv6xxx_sdio_pmu_wakeup,
    .load_fw         = ssv6xxx_sdio_load_firmware,
    .cmd52_read      = ssv6xxx_sdio_cmd52_read,
    .cmd52_write     = ssv6xxx_sdio_cmd52_write,
    .support_scatter = ssv6xxx_sdio_support_scatter,
    .rw_scatter      = ssv6xxx_sdio_rw_scatter,
    .is_ready        = ssv6xxx_is_ready,
    .write_sram     = ssv6xxx_sdio_write_sram,    
};


#ifdef CONFIG_PCIEASPM
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/mmc/sdhci.h>
// Disable PCIe power saving mode to ensure correct operation of SDIO interface.
static int cabrio_sdio_pm_check(struct sdio_func *func)
{
	struct mmc_card *card = func->card;
	struct mmc_host	*host = card->host;
	struct sdhci_host *sd_host = mmc_priv(host);
	struct pci_dev *pci_dev = NULL;

	// If the corresponding SDIO host is PCI device. Find PCI device
	// with "sdhci-pci" driver and disable its ASPM function.

	// Check if the SDIO host is a PCI device.
	if (strcmp(sd_host->hw_name, "PCI"))
	{
		dev_info(&func->dev, "SDIO host is not PCI device, but \"%s\".", sd_host->hw_name);
		return 0;
	}

	// Find the PCI device of SDHCI host
	for_each_pci_dev(pci_dev) {
		if (   ((pci_dev->class >> 8) != PCI_CLASS_SYSTEM_SDHCI)
			&& (   (pci_dev->driver == NULL)
				|| (strcmp(pci_dev->driver->name, "sdhci-pci") != 0)))
			continue;
		// Disable ASPM if it is a PCIe device.
		if (pci_is_pcie(pci_dev)) {
			u8 aspm;
			int pos;

			pos = pci_pcie_cap(pci_dev);
        	if (pos) {
        	    struct pci_dev *parent = pci_dev->bus->self;

        	    pci_read_config_byte(pci_dev, pos + PCI_EXP_LNKCTL, &aspm);
        	    aspm &= ~(PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1);
        	    pci_write_config_byte(pci_dev, pos + PCI_EXP_LNKCTL, aspm);

        	    pos = pci_pcie_cap(parent);
        	    pci_read_config_byte(parent, pos + PCI_EXP_LNKCTL, &aspm);
        	    aspm &= ~(PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1);
        	    pci_write_config_byte(parent, pos + PCI_EXP_LNKCTL, aspm);

        	    dev_info(&pci_dev->dev, "Clear PCI-E device and its parent link state L0S and L1 and CLKPM.\n");
        	}
		}
	}
	return 0;
}
#endif // CONFIG_PCIEASPM



static int ssv6xxx_sdio_power_on(struct ssv6xxx_platform_data * pdata, struct sdio_func *func)
{
	
	int ret = 0;

	if (pdata->is_enabled == true)
		return 0;

    printk("ssv6xxx_sdio_power_on\n");

	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	sdio_release_host(func);
	
	
	if (ret) {
		printk("Unable to enable sdio func: %d)\n", ret);		
		return ret;
	}
	
	/*
	 * Wait for hardware to initialise. It should take a lot less than
	 * 10 ms but let's be conservative here.
	 */
	msleep(10);

	pdata->is_enabled = true;

	return ret;
}

static int ssv6xxx_sdio_power_off(struct ssv6xxx_platform_data * pdata, struct sdio_func *func)
{
	int ret;

	if (pdata->is_enabled == false)
		return 0;
    
    printk("ssv6xxx_sdio_power_off\n");

	/* Disable the card */
	sdio_claim_host(func);
	ret = sdio_disable_func(func);
	sdio_release_host(func);

	if (ret)
		return ret;

	pdata->is_enabled = false;

	return ret;
}

#if SSV_XTAL==24
static int ssv6xxx_sdio_write_53(unsigned int regIOPort ,struct sdio_func *func, u32 addr, u32 buf)
{
    int ret=-1;
    u8 data[8];

    // 4 bytes address
    data[0] = (addr >> ( 0 )) &0xff;
    data[1] = (addr >> ( 8 )) &0xff;
    data[2] = (addr >> ( 16 )) &0xff;
    data[3] = (addr >> ( 24 )) &0xff;

    // 4 bytes data
    data[4] = (buf >> ( 0 )) &0xff;
    data[5] = (buf >> ( 8 )) &0xff;
    data[6] = (buf >> ( 16 )) &0xff;
    data[7] = (buf >> ( 24 )) &0xff;

    //8 byte ( 4 bytes address , 4 bytes data )
    ret = sdio_memcpy_toio(func, regIOPort, data, 8);

    return ret;
}
#endif // SSV_XTAL

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
static int __devinit ssv6xxx_sdio_probe(struct sdio_func *func,
        const struct sdio_device_id *id)
#else
static int ssv6xxx_sdio_probe(struct sdio_func *func,
        const struct sdio_device_id *id)
#endif
{
    struct ssv6xxx_platform_data *pwlan_data = &wlan_data;
    struct ssv6xxx_sdio_glue     *glue;

#if SSV_XTAL==24
    struct mmc_card *card = func->card;
    struct mmc_host *host = card->host;
#endif // SSV_XTAL

    //struct resource res[1];
    //mmc_pm_flag_t mmcflags;
    int ret = -ENOMEM;
    const char *chip_family = "ssv6200";

    if (ssv_devicetype != 0) {
        printk(KERN_INFO "Not using SSV6200 normal SDIO driver.\n");
        return -ENODEV;
    }

    printk(KERN_INFO "=======================================\n");
    printk(KERN_INFO "==           RUN SDIO                ==\n");
    printk(KERN_INFO "=======================================\n");
    

    /* We are only able to handle the wlan function */
    if (func->num != 0x01)
        return -ENODEV;

    glue = kzalloc(sizeof(*glue), GFP_KERNEL);
    if (!glue)
    {
        dev_err(&func->dev, "can't allocate glue\n");
        goto out;
    }

    /* Tell PM core that we don't need the card to be powered now */
    //pm_runtime_put_noidle(&func->dev);
#if SSV_XTAL==24

    //Setting SDIO to 25M
    host->ios.clock = 25000000;
    host->ops->set_ios(host,&host->ios);
    mdelay(10);

#endif // SSV_XTAL

#ifdef  CONFIG_FW_ALIGNMENT_CHECK
    glue->dmaSkb=__dev_alloc_skb(SDIO_DMA_BUFFER_LEN , GFP_KERNEL);
#endif
#ifdef  CONFIG_PM
    glue->cmd_skb=__dev_alloc_skb(SDIO_COMMAND_BUFFER_LEN , GFP_KERNEL);
#endif
    memset(pwlan_data, 0, sizeof(struct ssv6xxx_platform_data));
   
    atomic_set(&pwlan_data->irq_handling, 0);
    
    glue->dev = &func->dev;
    //init_waitqueue_head(&glue->irq_wq);

    /* Grab access to FN0 for ELP reg. */
    func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

    /* Use block mode for transferring over one block size of data */
    func->card->quirks |= MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;

    glue->dev_ready = true;
    /* if sdio can keep power while host is suspended, enable wow */
    //mmcflags = sdio_get_host_pm_caps(func);
    //dev_err(glue->dev, "sdio PM caps = 0x%x\n", mmcflags);
/*
    if (mmcflags & MMC_PM_KEEP_POWER)
        pwlan_data->pwr_in_suspend = true;
*/
    //store sdio vendor/device id
    pwlan_data->vendor = func->vendor;
    pwlan_data->device = func->device;

    dev_err(glue->dev, "vendor = 0x%x device = 0x%x\n",  pwlan_data->vendor,pwlan_data->device);
    #ifdef CONFIG_PCIEASPM
    cabrio_sdio_pm_check(func);
    #endif // CONFIG_PCIEASPM
    pwlan_data->ops = &sdio_ops;

    sdio_set_drvdata(func, glue);

    ssv6xxx_sdio_power_on(pwlan_data, func);
    
    ssv6xxx_sdio_read_parameter(func,glue);
#if SSV_XTAL==24
    sdio_claim_host(func);
    ssv6xxx_sdio_write_53(glue->regIOPort,func,ADR_BRG_SW_RST,0x2);
    ssv6xxx_sdio_write_53(glue->regIOPort,func,0xCE010038,0x0003E07C);
    ssv6xxx_sdio_write_53(glue->regIOPort,func,0xCE010060,0x00406000);
    ssv6xxx_sdio_write_53(glue->regIOPort,func,0xCE01009C,0x00000028);

    ssv6xxx_sdio_write_53(glue->regIOPort,func,0xCE0100A0,0x00);
    ssv6xxx_sdio_write_53(glue->regIOPort,func,ADR_BRG_SW_RST,0x2);
	sdio_release_host(func);
    mdelay(10);
#endif

    /* Tell PM core that we don't need the card to be powered now */
    //pm_runtime_put_noidle(&func->dev);

    glue->core = platform_device_alloc(chip_family, -1);
    if (!glue->core)
    {
        dev_err(glue->dev, "can't allocate platform_device");
        ret = -ENOMEM;
        goto out_free_glue;
    }

    glue->core->dev.parent = &func->dev;

    //dev_err(glue->dev, "sdio ssv6xxx_sdio_probe device[%08x] parent[%08x]\n",&glue->core->dev,&func->dev);

    /*
    	memset(res, 0x00, sizeof(res));

    	res[0].start = pwlan_data->irq;
    	res[0].flags = IORESOURCE_IRQ;
    	res[0].name = "irq";

    	ret = platform_device_add_resources(glue->core, res, ARRAY_SIZE(res));
    	if (ret) {
    		dev_err(glue->dev, "can't add resources\n");
    		goto out_dev_put;
    	}
    */    
    ret = platform_device_add_data(glue->core, pwlan_data,
                                   sizeof(*pwlan_data));
    if (ret)
    {
        dev_err(glue->dev, "can't add platform data\n");
        goto out_dev_put;
    }

    ret = platform_device_add(glue->core);
    if (ret)
    {
        dev_err(glue->dev, "can't add platform device\n");
        goto out_dev_put;
    }

    ssv6xxx_sdio_irq_setmask(&glue->core->dev,0xff);

#if 0
    ssv6xxx_sdio_irq_enable(&glue->core->dev);
#else


#endif

#if 0
    glue->dev->platform_data = (void *)pwlan_data;
    ret = ssv6xxx_dev_probe(glue->dev);
    if (ret)
    {
        dev_err(glue->dev, "failed to initial ssv6xxx device !!\n");
        platform_device_del(glue->core);
        goto out_dev_put;
    }
#endif

    ssv6xxx_ifdebug_info[0] = (void *)&glue->core->dev;
    ssv6xxx_ifdebug_info[1] = (void *)glue->core;
    ssv6xxx_ifdebug_info[2] = (void *)&sdio_ops;
    return 0;

out_dev_put:
    platform_device_put(glue->core);

out_free_glue:
    kfree(glue);

out:
    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
static void __devexit ssv6xxx_sdio_remove(struct sdio_func *func)
#else
static void ssv6xxx_sdio_remove(struct sdio_func *func)
#endif
{
    struct ssv6xxx_sdio_glue *glue = sdio_get_drvdata(func);
    struct ssv6xxx_platform_data *pwlan_data = &wlan_data;
#if SSV_XTAL==24
    struct mmc_card *card = func->card;
    struct mmc_host *host = card->host;
#endif // SSV_XTAL

    printk("ssv6xxx_sdio_remove..........\n");
    
    /* Undo decrement done above in ssv6xxx_probe */
    //pm_runtime_get_noresume(&func->dev);

    
    if ( glue )
    {
        // Remove IRQ handler once card is removed.
        printk("ssv6xxx_sdio_remove - ssv6xxx_sdio_irq_disable\n");
        ssv6xxx_sdio_irq_disable(&glue->core->dev,false);
        // Mark device is not ready for access.
        glue->dev_ready = false;
#if 0
        ssv6xxx_dev_remove(glue->dev);
#endif
#if SSV_XTAL==24
        //Setting SDIO to 25M
        host->ios.clock = 25000000;
        host->ops->set_ios(host,&host->ios);
        mdelay(10);
#endif // SSV_XTAL

#ifdef  CONFIG_FW_ALIGNMENT_CHECK
		if(glue->dmaSkb != NULL)
        	dev_kfree_skb(glue->dmaSkb);
#endif
#ifdef  CONFIG_PM
        if(glue->cmd_skb != NULL)
            dev_kfree_skb(glue->cmd_skb);
#endif
        printk("ssv6xxx_sdio_remove - disable mask\n");
        ssv6xxx_sdio_irq_setmask(&glue->core->dev,0xff);

        ssv6xxx_sdio_power_off(pwlan_data, func);

        printk("platform_device_del \n");
        platform_device_del(glue->core);

        printk("platform_device_put \n");
        platform_device_put(glue->core);
                   
        kfree(glue);
    }
    sdio_set_drvdata(func, NULL);

    printk("ssv6xxx_sdio_remove leave..........\n");
}

#ifdef CONFIG_PM
static int ssv6xxx_sdio_trigger_pmu(struct device *dev)
{
    struct sdio_func *func = dev_to_sdio_func(dev);
    struct ssv6xxx_sdio_glue *glue = sdio_get_drvdata(func);
    struct cfg_host_cmd *host_cmd;
    int writesize;
    int ret = 0;
    void *tempPointer;

#ifdef SSV_WAKEUP_HOST
    if(ssv6xxx_sdio_write_reg(dev, ADR_RX_FLOW_MNG, M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8)));
    if(ssv6xxx_sdio_write_reg(dev, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8)));
    if(ssv6xxx_sdio_write_reg(dev, ADR_MRX_FLT_TB0+6*4, (sc->mac_deci_tbl[6]|1)));//ACTION_DO_NOTHING, FRAME_DROP 0x11F8  /* 0 001000 111111 00 1 */       //Beacon
#else
    if(ssv6xxx_sdio_write_reg(dev, ADR_RX_FLOW_MNG, M_ENG_MACRX|(M_ENG_TRASH_CAN<<4)));
    if(ssv6xxx_sdio_write_reg(dev, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_TRASH_CAN<<4)));
    if(ssv6xxx_sdio_write_reg(dev, ADR_RX_FLOW_CTRL, M_ENG_MACRX|(M_ENG_TRASH_CAN<<4)));
#endif

    host_cmd = (struct cfg_host_cmd *)glue->cmd_skb->data;
    host_cmd->c_type = HOST_CMD;
    host_cmd->RSVD0 = 0;
    host_cmd->h_cmd = (u8)SSV6XXX_HOST_CMD_PS;
    host_cmd->len = sizeof(struct cfg_host_cmd);
#ifdef SSV_WAKEUP_HOST
    host_cmd->dummy = sc->ps_aid;
#else
	host_cmd->dummy = 0;
#endif

    {
        tempPointer = glue->cmd_skb->data;
        sdio_claim_host(func);
        writesize = sdio_align_size(func,sizeof(struct cfg_host_cmd));

        do
        {
            ret = sdio_memcpy_toio(func, glue->dataIOPort, tempPointer, writesize);
            if ( ret == -EILSEQ || ret == -ETIMEDOUT )
            {
                ret = -1;
                break;
            }
            else
            {
                if(ret)
                    dev_err(glue->dev,"Unexpected return value ret=[%d]\n",ret);
            }
        }
        while( ret == -EILSEQ || ret == -ETIMEDOUT);
        sdio_release_host(func);

        if (ret)
            dev_err(glue->dev, "sdio write failed (%d)\n", ret);
    }
    return ret;
}

static int ssv6xxx_sdio_suspend(struct device *dev)
{
    /* Tell MMC/SDIO core it's OK to power down the card
     * (if it isn't already), but not to remove it completely */
    struct sdio_func *func = dev_to_sdio_func(dev);
    //struct ssv6xxx_sdio_glue *glue = sdio_get_drvdata(func);
    //struct ssv_softc *softc = platform_get_drvdata(glue->core);
     mmc_pm_flag_t flags = sdio_get_host_pm_caps(func);
    int ret = 0;

    //dev_dbg(dev, "ssvcabrio suspend. wow_enabled: %d\n",
    //        softc->wow_enabled);

    /* check whether sdio should keep power */

    dev_info(dev, "%s: suspend: PM flags = 0x%x\n",
             sdio_func_id(func), flags);

    ret = ssv6xxx_sdio_trigger_pmu(dev);
    if (ret)
        printk("ssv6xxx_sdio_trigger_pmu fail!!\n");

    if (!(flags & MMC_PM_KEEP_POWER))
    {
    	dev_err(dev, "%s: cannot remain alive while host is suspended\n",
                sdio_func_id(func));
    	//return -ENOSYS;
    }

    ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
    if (ret)
    	return ret;

    // ret = lbs_suspend(card->priv);
    //if (ret)
    //	return ret;

#ifdef SSV_WAKEUP_HOST
    ret = sdio_set_host_pm_flags(func, MMC_PM_WAKE_SDIO_IRQ);
#endif
 
    #if 0
    if (softc->wow_enabled)
    {
        sdio_flags = sdio_get_host_pm_caps(func);

        if (!(sdio_flags & MMC_PM_KEEP_POWER))
        {
            dev_err(dev, "can't keep power while host "
                    "is suspended\n");
            ret = -EINVAL;
            goto out;
        }

        /* keep power while host suspended */
        ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
        if (ret)
        {
            dev_err(dev, "error while trying to keep power\n");
            goto out;
        }
    }else{
        ssv6xxx_sdio_irq_disable(&glue->core->dev,true);
    }
#endif    
//out:
    return ret;
}


static int ssv6xxx_sdio_resume(struct device *dev)
{
    //struct sdio_func *func = dev_to_sdio_func(dev);
    //struct ssv6xxx_sdio_glue *glue = sdio_get_drvdata(func);
    int err_ret;
    struct sdio_func *func;

    printk("ssv6xxx_sdio_resume\n");
    {
        func = dev_to_sdio_func(dev);
        sdio_claim_host(func);
        //sdio_writeb(func,0x88, REG_PMU_WAKEUP, &err_ret); //FPGA
        sdio_writeb(func,0x01, REG_PMU_WAKEUP, &err_ret);
#if SSV_VOLT_REGULATOR == VOLT_LDO_REGULATOR
		mdelay(20);
#elif SSV_VOLT_REGULATOR == VOLT_DCDC_CONVERT
		mdelay(120);
#else
# error "Please redefine SSV_VOLT_REGULATOR!!"
#endif
        sdio_writeb(func,0x00, REG_PMU_WAKEUP, &err_ret);
        sdio_release_host(func);
    }

	/*Why ??*/
	//ssv6xxx_sdio_read_parameter(func,glue);
    
    return 0;
}

static const struct dev_pm_ops ssv6xxx_sdio_pm_ops =
{
    .suspend    = ssv6xxx_sdio_suspend,
    .resume     = ssv6xxx_sdio_resume,
};
#endif

struct sdio_driver ssv6xxx_sdio_driver =
{
    .name		= "SSV6XXX_SDIO",
    .id_table	= ssv6xxx_sdio_devices,
    .probe		= ssv6xxx_sdio_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    .remove		= __devexit_p(ssv6xxx_sdio_remove),
#else
    .remove     = ssv6xxx_sdio_remove,
#endif
    
#ifdef CONFIG_PM
    .drv = {
        .pm = &ssv6xxx_sdio_pm_ops,
    },
#endif
};
EXPORT_SYMBOL(ssv6xxx_sdio_driver);

#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
int ssv6xxx_sdio_init(void)
#else
static int __init ssv6xxx_sdio_init(void)
#endif
{
    printk(KERN_INFO "ssv6xxx_sdio_init\n");
    return sdio_register_driver(&ssv6xxx_sdio_driver);
}

#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
void ssv6xxx_sdio_exit(void)
#else
static void __exit ssv6xxx_sdio_exit(void)
#endif
{
    printk(KERN_INFO "ssv6xxx_sdio_exit\n");
    sdio_unregister_driver(&ssv6xxx_sdio_driver);
}
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
EXPORT_SYMBOL(ssv6xxx_sdio_init);
EXPORT_SYMBOL(ssv6xxx_sdio_exit);
#else
module_init(ssv6xxx_sdio_init);
module_exit(ssv6xxx_sdio_exit);
#endif
MODULE_LICENSE("GPL");
