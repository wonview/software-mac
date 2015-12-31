#include <linux/version.h>
#include <ssv6200.h>
#include "dev.h"
#include "ap.h"
#include "sec.h"
#include "ssv_rc_common.h"
#include "ssv_ht_rc.h"

extern struct ieee80211_ops ssv6200_ops;

#define BA_WAIT_TIMEOUT           (800)
#define AMPDU_BA_FRAME_LEN        (68) // 28 BA + 36 RX DESC + 4 PHY PADDING
#define ampdu_skb_hdr(skb)        ((struct ieee80211_hdr*)((u8*)((skb)->data)+AMPDU_DELIMITER_LEN))
#define ampdu_skb_ssn(skb)        ((ampdu_skb_hdr(skb)->seq_ctrl)>>SSV_SEQ_NUM_SHIFT)
#define ampdu_hdr_ssn(hdr)        ((hdr)->seq_ctrl>>SSV_SEQ_NUM_SHIFT)

#if 0
#define prn_aggr_dbg(fmt, ...) \
    do { \
        printk(KERN_DEBUG fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define prn_aggr_dbg(fmt, ...)
#endif

#if 0
#define prn_aggr_err(fmt, ...) \
    do { \
        printk(KERN_ERR fmt, ##__VA_ARGS__);\
    } while (0)
#else
static void void_func(const char *fmt, ...)
{
}

#define prn_aggr_err(fmt, ...) \
    do { \
        void_func(KERN_ERR fmt, ##__VA_ARGS__);\
    } while (0)

#endif

#define get_tid_aggr_len(agg_len, tid_data) \
    ({ \
        u32 agg_max_num = (tid_data)->agg_num_max; \
        u32 to_agg_len = (agg_len); \
        (agg_len >= agg_max_num) ? agg_max_num : to_agg_len; \
    })

#define INDEX_PKT_BY_SSN(tid, ssn) \
    ((tid)->aggr_pkts[(ssn) % SSV_AMPDU_BA_WINDOW_SIZE])

#define NEXT_PKT_SN(sn) \
    ({ (sn + 1) % SSV_AMPDU_MAX_SSN; })

#define INC_PKT_SN(sn) \
    ({ \
        sn = NEXT_PKT_SN(sn); \
        sn; \
    })

static ssize_t ampdu_tx_mib_dump (struct ssv_sta_priv_data *ssv_sta_priv,
                                  char *mib_str, ssize_t length);
static struct sk_buff* _aggr_retry_mpdu (struct ssv_softc *sc,
                                         struct AMPDU_TID_st *cur_AMPDU_TID,
                                         struct sk_buff_head *retry_queue,
                                         u32 max_aggr_len);
static int _dump_ba_skb (char *buf, int buf_size, struct sk_buff *ba_skb);
static int _dump_BA_notification (char *buf,
                                  struct ampdu_ba_notify_data *ba_notification);
static struct sk_buff *_alloc_ampdu_skb (struct ssv_softc *sc,
                                         struct AMPDU_TID_st *ampdu_tid,
                                         u32 len);
static bool _sync_ampdu_pkt_arr (struct AMPDU_TID_st *ampdu_tid,
                                 struct sk_buff *ampdu_skb, bool retry);
static void _put_mpdu_to_ampdu (struct sk_buff *ampdu,
                                struct sk_buff *mpdu);
static void _add_ampdu_txinfo (struct ssv_softc *sc, struct sk_buff *ampdu_skb);
static u32 _flush_early_ampdu_q (struct ssv_softc *sc,
                                 struct AMPDU_TID_st *ampdu_tid);
static bool _is_skb_q_empty (struct ssv_softc *sc, struct sk_buff *skb);
static void _aggr_ampdu_tx_q (struct ieee80211_hw *hw,
                              struct AMPDU_TID_st *ampdu_tid);
static void _queue_early_ampdu (struct ssv_softc *sc,
                                struct AMPDU_TID_st *ampdu_tid,
                                struct sk_buff *ampdu_skb);
static int _mark_skb_retry (struct SKB_info_st *skb_info, struct sk_buff *skb);

/*
 *     c0  c1  c2  c3  c4  c5  c6  c7
 *             a6  a7                             (a << 4) & 0x30
 *     a0  a1  a2  a3  a4  a5  a6  a7             (a)
 *     a7  a0  a1  a2  a3  a4  a5  a6             (a >> 1) | (a << 7)
 *     a6  a6  a0  a1  a2  a3  a4  a5             (a >> 2) | ((a & 2) ? 0xC0 : 0)
 *     b6  b6  b0  b1  b2  b3  b4  b5             (b >> 2) | ((b & 2) ? 0xC0 : 0)
 *     b7  b0  b1  b2  b3  b4  b5  b6             (b >> 1) | (b << 7)
 *     b0  b1  b2  b3  b4  b5  b6  b7             (b)
 *             b6  b7                             (b << 4) & 0x30
 *
 */
static u8 _cal_ampdu_delm_half_crc (u8 value)
{
    u32 c32 = value, v32 = value;
    c32 ^= (v32 >> 1) | (v32 << 7);
    c32 ^= (v32 >> 2);
    if (v32 & 2)
        c32 ^= (0xC0);
    c32 ^= ((v32 << 4) & 0x30);
    return (u8) c32;
} // end of - _cal_ampdu_delm_half_crc -

static u8 _cal_ampdu_delm_crc (u8 *pointer)
{
    u8 crc = 0xCF;
    crc ^= _cal_ampdu_delm_half_crc(*pointer++);
    crc = _cal_ampdu_delm_half_crc(crc) ^ _cal_ampdu_delm_half_crc(*pointer);
    return ~crc;
} // end of - _cal_ampdu_delm_crc -

static bool ssv6200_ampdu_add_delimiter_and_crc32 (struct sk_buff *mpdu)
{
    p_AMPDU_DELIMITER delimiter_p;
    struct ieee80211_hdr *mpdu_hdr;
    int ret;
    u32 orig_mpdu_len = mpdu->len;
    u32 pad = (4 - (orig_mpdu_len % 4)) % 4;

    // fill duration ==============================================
    // data rate : mcs_5, 6, 7 or higher (else disable ampdu).
    // ======================================================
    mpdu_hdr = (struct ieee80211_hdr*) (mpdu->data);
    mpdu_hdr->duration_id = AMPDU_TX_NAV_MCS_567; // for mcs_5, 6, 7 or higher.

    // put FCS length =============================================
    ret = skb_padto(mpdu, mpdu->len + (AMPDU_FCS_LEN + pad));
    if (ret)
    {
        printk(KERN_ERR "Failed to extand skb for aggregation.");
        return false;
    }

    skb_put(mpdu, AMPDU_FCS_LEN + pad);

    // delimiter =================================================
    skb_push(mpdu, AMPDU_DELIMITER_LEN);

    delimiter_p = (p_AMPDU_DELIMITER) mpdu->data;

    delimiter_p->reserved = 0;
    delimiter_p->length = orig_mpdu_len + AMPDU_FCS_LEN;
    delimiter_p->signature = AMPDU_SIGNATURE;
    delimiter_p->crc = _cal_ampdu_delm_crc((u8*) (delimiter_p));

    return true;
} // end of - ssv6200_ampdu_add_delimiter_and_crc32 -


static void ssv6200_ampdu_hw_init (struct ieee80211_hw *hw)
{
    // Enable HW CRC32 for AMPDU
    struct ssv_softc *sc = hw->priv;
    u32 temp32;

    // Enable HW_AUTO_CRC_32 ======================================
    SMAC_REG_READ(sc->sh, ADR_MTX_MISC_EN, &temp32);
    temp32 |= (0x1 << MTX_AMPDU_CRC_AUTO_SFT);
    SMAC_REG_WRITE(sc->sh, ADR_MTX_MISC_EN, temp32);
    SMAC_REG_READ(sc->sh, ADR_MTX_MISC_EN, &temp32);
} // end of - ssv6200_ampdu_hw_init -

// Before sending to HCI, i.e. the AMPDU should be valid in current BA window,
// update AMPDU TX packet array.
bool _sync_ampdu_pkt_arr (struct AMPDU_TID_st *ampdu_tid, struct sk_buff *ampdu,
                          bool retry)
{
    struct sk_buff **pp_aggr_pkt;
    struct sk_buff *p_aggr_pkt;
    unsigned long flags;
    struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *) ampdu->head;
    struct sk_buff *mpdu;
    u32 first_ssn = SSV_ILLEGAL_SN;
    u32 old_aggr_pkt_num;
    u32 old_baw_head;
    u32 sync_num = skb_queue_len(&ampdu_hdr->mpdu_q);
    bool ret = true;

    spin_lock_irqsave(&ampdu_tid->pkt_array_lock, flags);
    //spin_lock_bh(&ampdu_tid->pkt_array_lock);

    old_baw_head = ampdu_tid->ssv_baw_head;
    old_aggr_pkt_num = ampdu_tid->aggr_pkt_num;

    ampdu_tid->mib.ampdu_mib_ampdu_counter += 1;
    ampdu_tid->mib.ampdu_mib_dist[sync_num] += 1;

    do
    {
        // Ensure there is room in BA window for this AMPDU by checking
        // if the slot for the last one MPDU is empty.
        if (!retry)
        {
            ampdu_tid->mib.ampdu_mib_mpdu_counter += sync_num;
            mpdu = skb_peek_tail(&ampdu_hdr->mpdu_q);
            if (mpdu == NULL)
            {
                ret = false;
                break;
            }
            else
            {
                u32 ssn = ampdu_skb_ssn(mpdu);

                p_aggr_pkt = INDEX_PKT_BY_SSN(ampdu_tid, ssn);
                if (p_aggr_pkt != NULL)
                {
                    char msg[256];
                    u32 sn = ampdu_skb_ssn(mpdu);
                    skb_queue_walk(&ampdu_hdr->mpdu_q, mpdu)
                    {
                        sn = ampdu_skb_ssn(mpdu);
                        sprintf(msg, " %d", sn);
                    }

                    prn_aggr_err("ES %d -> %d (%s)\n",
                                 ssn, ampdu_skb_ssn(p_aggr_pkt), msg);
                    ret = false;
                    break;
                }
            }
        }
        else
            ampdu_tid->mib.ampdu_mib_aggr_retry_counter += 1;

        skb_queue_walk(&ampdu_hdr->mpdu_q, mpdu)
        {
            u32 ssn = ampdu_skb_ssn(mpdu);
            SKB_info *mpdu_skb_info = (SKB_info *) (mpdu->head);

            if (first_ssn == SSV_ILLEGAL_SN)
                first_ssn = ssn;

            pp_aggr_pkt = &INDEX_PKT_BY_SSN(ampdu_tid, ssn);
            p_aggr_pkt = *pp_aggr_pkt;
            *pp_aggr_pkt = mpdu;
            //if (mpdu_skb_info->ampdu_tx_status == AMPDU_ST_NON_AMPDU)
            if (!retry)
                ampdu_tid->aggr_pkt_num++;
            mpdu_skb_info->ampdu_tx_status = AMPDU_ST_AGGREGATED;

            if (ampdu_tid->ssv_baw_head == SSV_ILLEGAL_SN)
            {
                ampdu_tid->ssv_baw_head = ssn;
                //prn_aggr_err("S %d\n", ssn);
            }
            // Debug: SKB should not be overwriten.
            if ((p_aggr_pkt != NULL) && (mpdu != p_aggr_pkt))
                prn_aggr_err(
                        "%d -> %d (H%d, N%d, Q%d)\n",
                        ssn, ampdu_skb_ssn(p_aggr_pkt), old_baw_head, old_aggr_pkt_num, sync_num);
        }
    } while (0);

    spin_unlock_irqrestore(&ampdu_tid->pkt_array_lock, flags);
    //spin_unlock_bh(&ampdu_tid->pkt_array_lock);

    #if 1
    {
    u32 page_count = (ampdu->len + SSV6200_ALLOC_RSVD);
    if (page_count & HW_MMU_PAGE_MASK)
        page_count = (page_count >> HW_MMU_PAGE_SHIFT) + 1;
    else page_count = page_count >> HW_MMU_PAGE_SHIFT;

    if (page_count > (SSV6200_PAGE_TX_THRESHOLD / 2))
        printk(KERN_ERR "AMPDU requires pages %d(%d-%d-%d) exceeds resource limit %d.\n",
               page_count, ampdu->len, ampdu_hdr->max_size, ampdu_hdr->size,
               (SSV6200_PAGE_TX_THRESHOLD / 2));
    }
    #endif
    return ret;
} // end of - _sync_ampdu_pkt_arr -

struct sk_buff* _aggr_retry_mpdu (struct ssv_softc *sc,
                                  struct AMPDU_TID_st *ampdu_tid,
                                  struct sk_buff_head *retry_queue,
                                  u32 max_aggr_len)
{
    struct sk_buff         *retry_mpdu;
    struct sk_buff         *new_ampdu_skb;
    u32                     num_retry_mpdu;
    u32                     temp_i;
    u32                     total_skb_size;
    unsigned long           flags;
    struct ampdu_hdr_st    *ampdu_hdr;

#if 0
    if (cur_AMPDU_TID->ssv_baw_head == SSV_ILLEGAL_SN)
    {
        struct sk_buff *skb = skb_peek(ampdu_skb_retry_queue);
        prn_aggr_err("Rr %d\n", (skb == NULL) ? (-1) : ampdu_skb_ssn(skb));
        return NULL;
    }
#else
    BUG_ON(ampdu_tid->ssv_baw_head == SSV_ILLEGAL_SN);
#endif // 0

    num_retry_mpdu = skb_queue_len(retry_queue);
    if (num_retry_mpdu == 0)
        return NULL;

    new_ampdu_skb = _alloc_ampdu_skb(sc, ampdu_tid, max_aggr_len);

    if (new_ampdu_skb == 0)
        return NULL;

    ampdu_hdr = (struct ampdu_hdr_st *)new_ampdu_skb->head;

    total_skb_size = 0;

    spin_lock_irqsave(&retry_queue->lock, flags);

    for (temp_i = 0; temp_i < ampdu_tid->agg_num_max; temp_i++)
    {
        struct ieee80211_hdr   *mpdu_hdr;
        u16                     mpdu_sn;
        int                     diff;
        u32                     new_total_skb_size;

        //ampdu_skb = skb_dequeue(ampdu_skb_output_queue);
        retry_mpdu = skb_peek(retry_queue);

        if (retry_mpdu == NULL)
        {
            break;
            //continue;
        }

        mpdu_hdr = ampdu_skb_hdr(retry_mpdu);
        mpdu_sn = ampdu_hdr_ssn(mpdu_hdr);

        diff = SSV_AMPDU_SN_a_minus_b(ampdu_tid->ssv_baw_head, mpdu_sn);
        if ((ampdu_tid->ssv_baw_head != SSV_ILLEGAL_SN)
            && (diff > 0)
            && (diff <= ampdu_tid->ssv_baw_size))
        {
            struct SKB_info_st *skb_info;
            // release skb
            prn_aggr_err("Z. release skb (s %d, h %d, d %d)\n",
                         mpdu_sn, ampdu_tid->ssv_baw_head, diff);

            skb_info = (struct SKB_info_st *) (retry_mpdu->head);
            skb_info->ampdu_tx_status = AMPDU_ST_DROPPED;

            ampdu_tid->mib.ampdu_mib_discard_counter++;
            continue;
        }

        new_total_skb_size = total_skb_size + retry_mpdu->len;
        // Maximum aggregation length can not exceeds allocated payload length.
        if (new_total_skb_size > ampdu_hdr->max_size)
            break;
        total_skb_size = new_total_skb_size;
#if 0
            if (ampdu_skb_retry_queue != NULL)
            prn_aggr_err("R %d\n", SerialNumber);
#endif // 0
        retry_mpdu = __skb_dequeue(retry_queue);
        _put_mpdu_to_ampdu(new_ampdu_skb, retry_mpdu);
        ampdu_tid->mib.ampdu_mib_retry_counter++;
        //__skb_queue_tail(&new_skb_tx_queue, ampdu_skb);
    }
    ampdu_tid->mib.ampdu_mib_aggr_retry_counter += 1;
    ampdu_tid->mib.ampdu_mib_dist[temp_i] += 1;

    spin_unlock_irqrestore(&retry_queue->lock, flags);
#if 0
    // Debug code
    {
    struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *)new_ampdu_skb->head;
    retry_mpdu = skb_peek(&ampdu_hdr->mpdu_q);
    dev_alert(sc->dev, "rA %d - %d\n", ampdu_skb_ssn(retry_mpdu),
              skb_queue_len(&ampdu_hdr->mpdu_q));
    }
#endif
    return new_ampdu_skb;
} // end of - _aggr_retry_mpdu -


static void _add_ampdu_txinfo (struct ssv_softc *sc, struct sk_buff *ampdu_skb)
{
    struct ssv6200_tx_desc *tx_desc;

    ssv6xxx_add_txinfo(sc, ampdu_skb);

    tx_desc = (struct ssv6200_tx_desc *) ampdu_skb->data;

    // Set TX report for firmware retry.
    tx_desc->tx_report = 1;

#if 0
    /* The definition of WORD_1: */
    tx_desc->len = ampdu_skb->len;
    tx_desc->c_type = M2_TXREQ;
    tx_desc->f80211 = 1;
    tx_desc->ht = 1;
    tx_desc->qos = 1;
    tx_desc->use_4addr = 0;
    tx_desc->security = 0;
    tx_desc->more_data = 0;
    tx_desc->stype_b5b4 = 0;
    tx_desc->extra_info = 0;

    /* The definition of WORD_3: */
    tx_desc->hdr_offset = sc->sh->cfg.txpb_offset;; // frame offset by LC    
    tx_desc->frag = 0;
    tx_desc->unicast = 1;
    tx_desc->hdr_len = 0;// don't care by LC
    tx_desc->RSVD_4 = 0;
    tx_desc->tx_burst = 0;
    tx_desc->ack_policy = 1;
    tx_desc->aggregation = 1;
    tx_desc->RSVD_1 = 0;
    tx_desc->do_rts_cts = 0;
    tx_desc->reason = 0;

    /* The definition of WORD_4: */
    tx_desc->payload_offset = tx_desc->hdr_offset + tx_desc->hdr_len; //:8;
    tx_desc->RSVD_4 = 0;
    tx_desc->RSVD_2 = 0;
    tx_desc->fCmdIdx = 0;
    tx_desc->wsid = 0;
    tx_desc->txq_idx = 0;
    tx_desc->TxF_ID = 0;

    /* The definition of WORD_5: */
    tx_desc->rts_cts_nav = 0;
    tx_desc->frame_consume_time = 0; //32 units
    tx_desc->crate_idx=0;

    /* The definition of WORD_6: */
    tx_desc->drate_idx = 22;
    tx_desc->dl_length = 56;
    tx_desc->RSVD_3 = 0;
#endif

#if 0
    // print txinfo

    if(ampdu_skb != 0)
    {
        u32 temp_i;
        u8* temp8_p = (u8*)ampdu_skb->data;

        ampdu_db_log("print txinfo.\n");

        for(temp_i=0; temp_i < 24; temp_i++)
        {
            ampdu_db_log_simple("%02x",*(u8*)(temp8_p + temp_i));

            if(((temp_i+1) % 4) == 0)
            {
                ampdu_db_log_simple("\n");
            }
        }
        ampdu_db_log_simple("\n");
    }
#endif

#if 0
    // print all skb

    if(ampdu_skb != 0)
    {
        u32 temp_i;
        u8* temp8_p = (u8*)ampdu_skb->data;

        ampdu_db_log("print all skb.\n");

        for(temp_i=0; temp_i < ampdu_skb->len; temp_i++)
        {
            ampdu_db_log_simple("%02x",*(u8*)(temp8_p + temp_i));

            if(((temp_i+1) % 4) == 0)
            {
                ampdu_db_log_simple("\n");
            }
        }
        ampdu_db_log_simple("\n");
    }
#endif // 0
} // end of - _add_ampdu_txinfo -

void _send_hci_skb (struct ssv_softc *sc, struct sk_buff *skb, u32 tx_flag)
{
    struct ssv6200_tx_desc *tx_desc = (struct ssv6200_tx_desc *)skb->data;

    int ret = AMPDU_HCI_SEND(sc->sh, skb, tx_desc->txq_idx, tx_flag);
#if 1
    if ((tx_desc->txq_idx > 3) && (ret <= 0))
    {
        prn_aggr_err("BUG!! %d %d\n", tx_desc->txq_idx, ret);
    }
#else
    BUG_ON(tx_desc->txq_idx>3 && ret<=0);
#endif
} // end of - _send_hci_skb -

static void ssv6200_ampdu_add_txinfo_and_send_HCI (struct ssv_softc *sc,
                                                   struct sk_buff *ampdu_skb,
                                                   u32 tx_flag)
{
    _add_ampdu_txinfo(sc, ampdu_skb);
    _send_hci_skb(sc, ampdu_skb, tx_flag);
}

static void ssv6200_ampdu_send_retry (
        struct ieee80211_hw *hw, AMPDU_TID *cur_ampdu_tid,
        struct sk_buff_head *ampdu_skb_retry_queue_p, bool send_aggr_tx)
{
    struct ssv_softc           *sc = hw->priv;
    struct sk_buff             *ampdu_retry_skb;
    u32                         ampdu_skb_retry_queue_len;
    u32                         max_agg_len;
    u16                         lowest_rate;
    struct fw_rc_retry_params   rates[SSV62XX_TX_MAX_RATES];


    ampdu_skb_retry_queue_len = skb_queue_len(ampdu_skb_retry_queue_p);

    if (ampdu_skb_retry_queue_len == 0)
        return;

    // Check current HT rate.
    ampdu_retry_skb = skb_peek(ampdu_skb_retry_queue_p);
    lowest_rate = ssv62xx_ht_rate_update(ampdu_retry_skb, sc, rates);
    max_agg_len = ampdu_max_transmit_length[lowest_rate];

    // Aggregate if aggregation is allowed.
    if (max_agg_len > 0)
    {
        u32 cur_ampdu_max_size = SSV_GET_MAX_AMPDU_SIZE(sc->sh);
        if (max_agg_len >= cur_ampdu_max_size)
            max_agg_len = cur_ampdu_max_size;

        while (ampdu_skb_retry_queue_len > 0)
        {
            // Set current HT rate to the first MPDU. So that the AMPDU would be
            // applied with this rate setting.
            struct sk_buff *retry_mpdu = skb_peek(ampdu_skb_retry_queue_p);
            SKB_info       *mpdu_skb_info = (SKB_info *)(retry_mpdu->head);
            mpdu_skb_info->lowest_rate = lowest_rate;
            memcpy(mpdu_skb_info->rates, rates, sizeof(rates));

            ampdu_retry_skb = _aggr_retry_mpdu(sc, cur_ampdu_tid, ampdu_skb_retry_queue_p,
                                               max_agg_len);

            if (ampdu_retry_skb != 0)
            {
                _sync_ampdu_pkt_arr(cur_ampdu_tid, ampdu_retry_skb, true);
                ssv6200_ampdu_add_txinfo_and_send_HCI(sc, ampdu_retry_skb,
                                                      AMPDU_HCI_SEND_HEAD_WITHOUT_FLOWCTRL);
            }
            else
            {
                prn_aggr_err("AMPDU retry failed.\n");
                //BUG_ON(1);
                return;
            }

            ampdu_skb_retry_queue_len = skb_queue_len(ampdu_skb_retry_queue_p);
        }
    }
    else
    {
        // Not able to aggregate. Send all queued retry frames with current rate.
        struct ieee80211_tx_rate rates[IEEE80211_TX_MAX_RATES];
        struct ieee80211_tx_info *info = IEEE80211_SKB_CB(ampdu_retry_skb);

        memcpy(rates, info->control.rates, sizeof(info->control.rates));

        while ((ampdu_retry_skb = __skb_dequeue_tail(ampdu_skb_retry_queue_p)) != NULL)
        {
            struct ieee80211_tx_info *info = IEEE80211_SKB_CB(ampdu_retry_skb);

            info->flags &= ~IEEE80211_TX_CTL_AMPDU;

            memcpy(info->control.rates, rates, sizeof(info->control.rates));

            ssv6xxx_update_txinfo(sc, ampdu_retry_skb);

            _send_hci_skb(sc, ampdu_retry_skb, AMPDU_HCI_SEND_HEAD_WITHOUT_FLOWCTRL);
        }
    }
} // end of - ssv6200_ampdu_send_retry -


void ssv6200_ampdu_init (struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;

    // init AMPDU HW =======================================
    ssv6200_ampdu_hw_init(hw);

    // init ampdu_tx_group_id ===========================
    sc->tx.ampdu_tx_group_id = 0;

#ifdef USE_ENCRYPT_WORK
    INIT_WORK(&sc->ampdu_tx_encry_work, encry_work);
    INIT_WORK(&sc->sync_hwkey_work, sync_hw_key_work);
#endif // USE_ENCRYPT_WORK
}

void ssv6200_ampdu_deinit (struct ieee80211_hw *hw)
{
    // Freddie ToDo: cancel_work_sync for ampdu_tx_encry_work and sync_hwkey_work?
    //cancel_work_sync(&sc->ampdu_tx_work);
    //cancel_work_sync(&sc->ampdu_retry_work);
}

void ssv6200_ampdu_release_skb (struct sk_buff *skb, struct ieee80211_hw *hw)
{
#if LINUX_VERSION_CODE >= 0x030400
    ieee80211_free_txskb(hw, skb);
#else
    dev_kfree_skb_any(skb);
#endif
}

#ifdef CONFIG_SSV6XXX_DEBUGFS

struct mib_dump_data {
    char    *prt_buff;
    size_t   buff_size;
    size_t   prt_len;
};


#define AMPDU_TX_MIB_SUMMARY_BUF_SIZE            (4096)
static ssize_t ampdu_tx_mib_summary_read (struct file *file,
                                          char __user *user_buf, size_t count,
                                          loff_t *ppos)
{
    struct ssv_sta_priv_data *ssv_sta_priv =
            (struct ssv_sta_priv_data *) file->private_data;
    char *summary_buf = kzalloc(AMPDU_TX_MIB_SUMMARY_BUF_SIZE, GFP_KERNEL);
    ssize_t summary_size;
    ssize_t ret;

    if (!summary_buf)
        return -ENOMEM;

    summary_size = ampdu_tx_mib_dump(ssv_sta_priv, summary_buf,
                                     AMPDU_TX_MIB_SUMMARY_BUF_SIZE);
    ret = simple_read_from_buffer(user_buf, count, ppos, summary_buf,
                                  summary_size);
    kfree(summary_buf);

    return ret;
}

static int ampdu_tx_mib_summary_open (struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;

    return 0;
}

static const struct file_operations mib_summary_fops = { .read =
        ampdu_tx_mib_summary_read, .open = ampdu_tx_mib_summary_open, };


static ssize_t ampdu_tx_tid_window_read (struct file *file,
                                          char __user *user_buf, size_t count,
                                          loff_t *ppos)
{
    struct AMPDU_TID_st *ampdu_tid = (struct AMPDU_TID_st *)file->private_data;

    char *summary_buf = kzalloc(AMPDU_TX_MIB_SUMMARY_BUF_SIZE, GFP_KERNEL);
    ssize_t ret;
    char *prn_ptr = summary_buf;
    int   prt_size;
    int   buf_size = AMPDU_TX_MIB_SUMMARY_BUF_SIZE;
    int   i;
    struct sk_buff *ba_skb, *tmp_ba_skb;

    if (!summary_buf)
        return -ENOMEM;

    prt_size = snprintf(prn_ptr, buf_size, "\nWMM_TID %d:\n"
                        "\tWindow:",
                        ampdu_tid->tidno);
    prn_ptr += prt_size;
    buf_size -= prt_size;

    for (i = 0; i < SSV_AMPDU_BA_WINDOW_SIZE; i++)
    {
        struct sk_buff *skb = ampdu_tid->aggr_pkts[i];
        if ((i % 8) == 0)
        {
            prt_size = snprintf(prn_ptr, buf_size, "\n\t\t");
            prn_ptr += prt_size;
            buf_size -= prt_size;
        }

        if (skb == NULL)
            prt_size = snprintf(prn_ptr, buf_size, " %s", "NULL ");
        else
        {
            struct SKB_info_st *skb_info = (struct SKB_info_st *)(skb->head);
            const char status_symbol[] = {'N', // AMPDU_ST_NON_AMPDU,    // Not an AMPDU frame or not being processed by AMPDU engine yet.
                                          'A', // AMPDU_ST_AGGREGATED,   // Aggregated into an AMPDU. No BA received.
                                          'S', // AMPDU_ST_SENT,         // HCI has sent AMPDU frame to WLAN adaptor.
                                          'R', // AMPDU_ST_RETRY,        // Need retry. (Missed acknowledgment)
                                          'P', // AMPDU_ST_DROPPED,      // Dropped MPDU. (After retry)
                                          'D'};// AMPDU_ST_DONE,         // TX done. (Acknowledged)
            prt_size = snprintf(prn_ptr, buf_size, " %4d%c", ampdu_skb_ssn(skb),
                                (  (skb_info->ampdu_tx_status <= AMPDU_ST_DONE)
                                 ? status_symbol[skb_info->ampdu_tx_status]
                                 : 'X'));
        }
        prn_ptr += prt_size;
        buf_size -= prt_size;
    }

    prt_size = snprintf(prn_ptr, buf_size, "\n\tEarly aggregated #: %d\n", ampdu_tid->early_aggr_skb_num);
    prn_ptr += prt_size;
    buf_size -= prt_size;

    prt_size = snprintf(prn_ptr, buf_size, "\tBAW skb #: %d\n", ampdu_tid->aggr_pkt_num);
    prn_ptr += prt_size;
    buf_size -= prt_size;

    prt_size = snprintf(prn_ptr, buf_size, "\tBAW head: %d\n", ampdu_tid->ssv_baw_head);
    prn_ptr += prt_size;
    buf_size -= prt_size;

    prt_size = snprintf(prn_ptr, buf_size, "\tState: %d\n", ampdu_tid->state);
    prn_ptr += prt_size;
    buf_size -= prt_size;
    
    prt_size = snprintf(prn_ptr, buf_size, "\tBA:\n");
    prn_ptr += prt_size;
    buf_size -= prt_size;
    
    skb_queue_walk_safe(&ampdu_tid->ba_q, ba_skb, tmp_ba_skb)
    {
        prt_size = _dump_ba_skb(prn_ptr, buf_size, ba_skb);
        prn_ptr += prt_size;
        buf_size -= prt_size;
    }

    buf_size = AMPDU_TX_MIB_SUMMARY_BUF_SIZE - buf_size;
    ret = simple_read_from_buffer(user_buf, count, ppos, summary_buf,
                                  buf_size);
    kfree(summary_buf);

    return ret;
}

static int ampdu_tx_tid_window_open (struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;

    return 0;
}

static const struct file_operations tid_window_fops = { .read =
        ampdu_tx_tid_window_read, .open = ampdu_tx_tid_window_open, };

static int ampdu_tx_mib_reset_open (struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;

    return 0;
}

static ssize_t ampdu_tx_mib_reset_read (struct file *file,
                                        char __user *user_buf, size_t count,
                                        loff_t *ppos)
{
    char                     *reset_buf = kzalloc(64, GFP_KERNEL);
    ssize_t                   ret;
    u32                       reset_size;

    if (!reset_buf)
        return -ENOMEM;

    reset_size = snprintf(reset_buf, 63, "%d", 0);

    ret = simple_read_from_buffer(user_buf, count, ppos, reset_buf,
                                  reset_size);
    kfree(reset_buf);

    return ret;
}

static ssize_t ampdu_tx_mib_reset_write (struct file *file,
                                         const char __user *buffer,
                                         size_t count,
                                         loff_t *pos)
{
    struct AMPDU_TID_st *ampdu_tid = (struct AMPDU_TID_st *)file->private_data;

    memset(&ampdu_tid->mib, 0, sizeof(struct AMPDU_MIB_st));

    return count;
}

static const struct file_operations mib_reset_fops
    = { .read = ampdu_tx_mib_reset_read,
        .open = ampdu_tx_mib_reset_open,
        .write = ampdu_tx_mib_reset_write};

static void ssv6200_ampdu_tx_init_debugfs (
        struct ssv_softc *sc, struct ssv_sta_priv_data *ssv_sta_priv)
{
    struct ssv_sta_info *sta_info = ssv_sta_priv->sta_info;
    int i;
    struct dentry *sta_debugfs_dir = sta_info->debugfs_dir;

    dev_info(sc->dev, "Creating AMPDU TX debugfs.\n");

    if (sta_debugfs_dir == NULL)
    {
        dev_err(sc->dev, "No STA debugfs.\n");
        return;
    }

    debugfs_create_file("ampdu_tx_summary", 00444, sta_debugfs_dir,
                        ssv_sta_priv, &mib_summary_fops);
    debugfs_create_u32("total_BA", 00666, sta_debugfs_dir,
                       &ssv_sta_priv->ampdu_mib_total_BA_counter);

    for (i = 0; i < WMM_TID_NUM; i++)
    {
        char debugfs_name[20];
        struct dentry *ampdu_tx_debugfs_dir;
        int j;
        struct AMPDU_TID_st *ampdu_tid = &ssv_sta_priv->ampdu_tid[i];
        struct AMPDU_MIB_st *ampdu_mib = &ampdu_tid->mib;

        snprintf(debugfs_name, sizeof(debugfs_name), "ampdu_tx_%d", i);
        ampdu_tx_debugfs_dir = debugfs_create_dir(debugfs_name,
                                                  sta_debugfs_dir);
        if (ampdu_tx_debugfs_dir == NULL)
        {
            dev_err(sc->dev,
                    "Failed to create debugfs for AMPDU TX TID %d: %s\n", i,
                    debugfs_name);
            continue;
        }

        ssv_sta_priv->ampdu_tid[i].debugfs_dir = ampdu_tx_debugfs_dir;

        debugfs_create_file("baw_status", 00444, ampdu_tx_debugfs_dir,
                            ampdu_tid, &tid_window_fops);

        debugfs_create_file("reset", 00666, ampdu_tx_debugfs_dir,
                            ampdu_tid, &mib_reset_fops);

        debugfs_create_u32("total", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_ampdu_counter);
        debugfs_create_u32("retry", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_retry_counter);
        debugfs_create_u32("aggr_retry", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_aggr_retry_counter);
        debugfs_create_u32("BAR", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_bar_counter);
        debugfs_create_u32("Discarded", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_discard_counter);
        debugfs_create_u32("BA", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_BA_counter);
        debugfs_create_u32("Pass", 00444, ampdu_tx_debugfs_dir,
                           &ampdu_mib->ampdu_mib_pass_counter);
        for (j = 0; j <= SSV_AMPDU_aggr_num_max; j++)
        {
            char dist_dbg_name[10];
            snprintf(dist_dbg_name, sizeof(dist_dbg_name), "aggr_%d", j);
            debugfs_create_u32(dist_dbg_name, 00444, ampdu_tx_debugfs_dir,
                               &ampdu_mib->ampdu_mib_dist[j]);
        }
        
        skb_queue_head_init(&ssv_sta_priv->ampdu_tid[i].ba_q);
    }
}
#endif // CONFIG_SSV6XXX_DEBUGFS

void ssv6200_ampdu_tx_add_sta (struct ieee80211_hw *hw,
                               struct ieee80211_sta *sta)
{
    struct ssv_sta_priv_data *ssv_sta_priv;
    struct ssv_softc *sc;
    u32 temp_i;

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    sc = (struct ssv_softc *) hw->priv;

    for (temp_i = 0; temp_i < WMM_TID_NUM; temp_i++)
    {
        ssv_sta_priv->ampdu_tid[temp_i].sta = sta;
        ssv_sta_priv->ampdu_tid[temp_i].state = AMPDU_STATE_STOP;
        // init spinlock =======================================
        spin_lock_init(
                &ssv_sta_priv->ampdu_tid[temp_i].ampdu_skb_tx_queue_lock);
        spin_lock_init(&ssv_sta_priv->ampdu_tid[temp_i].pkt_array_lock);
    }

#ifdef CONFIG_SSV6XXX_DEBUGFS
    ssv6200_ampdu_tx_init_debugfs(sc, ssv_sta_priv);
#endif // CONFIG_SSV6XXX_DEBUGFS
}

void ssv6200_ampdu_tx_start (u16 tid, struct ieee80211_sta *sta,
                             struct ieee80211_hw *hw, u16 *ssn)
{
    struct ssv_softc *sc = hw->priv;
    struct ssv_sta_priv_data *ssv_sta_priv;
    struct AMPDU_TID_st *ampdu_tid;
    int i;

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    ampdu_tid = &ssv_sta_priv->ampdu_tid[tid];

    // BAW control ======================================
    ampdu_tid->ssv_baw_head = SSV_ILLEGAL_SN;

    // start AMPDU_timer =====================================
#if 0
    if (list_empty(&sc->tx.ampdu_tx_que))
    {
        // ToDo: Rekey is by STA. Should not keep its status in sc.
        sc->ampdu_rekey_pause = AMPDU_REKEY_PAUSE_STOP;
    }
#endif
    // ampdu tx queue =====================================
#ifdef DEBUG_AMPDU_FLUSH
    printk(KERN_ERR "Adding %02X-%02X-%02X-%02X-%02X-%02X TID %d (%p).\n",
            sta->addr[0], sta->addr[1], sta->addr[2],
            sta->addr[3], sta->addr[4], sta->addr[5],
            ampdu_tid->tidno, ampdu_tid);
    {
        int j;
        for (j = 0; j <= MAX_TID; j++)
        {
            if (sc->tid[j] == 0)
                break;
        }

        if (j == MAX_TID)
        {
            printk(KERN_ERR "No room for new TID.\n");
        }
        else
            sc->tid[j] = ampdu_tid;
}
#endif // DEBUG_AMPDU_FLUSH
    list_add_tail_rcu(&ampdu_tid->list, &sc->tx.ampdu_tx_que);

    // init skb tx queqe =====================================
    skb_queue_head_init(&ampdu_tid->ampdu_skb_tx_queue);

#ifdef ENABLE_INCREMENTAL_AGGREGATION
    // init early AMPDU  queqe =====================================
    skb_queue_head_init(&ampdu_tid->early_aggr_ampdu_q);
    ampdu_tid->early_aggr_skb_num = 0;
#endif // ENABLE_INCREMENTAL_AGGREGATION
    skb_queue_head_init(&ampdu_tid->ampdu_skb_wait_encry_queue);

    // init skb tx retry queqe =====================================
    skb_queue_head_init(&ampdu_tid->retry_queue);
    // init skb tx release queqe =====================================
    skb_queue_head_init(&ampdu_tid->release_queue);

    for (i = 0;
            i < (sizeof(ampdu_tid->aggr_pkts) / sizeof(ampdu_tid->aggr_pkts[0]));
            i++)
        ampdu_tid->aggr_pkts[i] = 0;
    ampdu_tid->aggr_pkt_num = 0;

#ifdef ENABLE_AGGREGATE_IN_TIME
    ampdu_tid->cur_ampdu_pkt = _alloc_ampdu_skb(sc, ampdu_tid, 0);
#endif // ENABLE_AGGREGATE_IN_TIME
#ifdef AMPDU_CHECK_SKB_SEQNO
    ssv_sta_priv->ampdu_tid[tid].last_seqno = (-1);
#endif

    // init mib counter
    ssv_sta_priv->ampdu_mib_total_BA_counter = 0;
    memset(&ssv_sta_priv->ampdu_tid[tid].mib, 0, sizeof(struct AMPDU_MIB_st));

    ssv_sta_priv->ampdu_tid[tid].state = AMPDU_STATE_START;

    #ifdef CONFIG_SSV6XXX_DEBUGFS
    skb_queue_head_init(&ssv_sta_priv->ampdu_tid[tid].ba_q);
    #endif // CONFIG_SSV6XXX_DEBUGFS
} // end of - ssv6200_ampdu_tx_start -

void ssv6200_ampdu_tx_operation (u16 tid, struct ieee80211_sta *sta,
                                 struct ieee80211_hw *hw, u8 buffer_size)
{
    struct ssv_sta_priv_data *ssv_sta_priv;

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;

    ssv_sta_priv->ampdu_tid[tid].tidno = tid;
    ssv_sta_priv->ampdu_tid[tid].sta = sta;
    ssv_sta_priv->ampdu_tid[tid].agg_num_max = MAX_AGGR_NUM;
#if 1
    if (buffer_size > IEEE80211_MAX_AMPDU_BUF)
    {
        buffer_size = IEEE80211_MAX_AMPDU_BUF;
    }
    printk("ssv6200_ampdu_tx_operation:buffer_size=%d\n", buffer_size);
    ssv_sta_priv->ampdu_tid[tid].ssv_baw_size = SSV_AMPDU_WINDOW_SIZE; //buffer_size;
#else
    ssv_sta_priv->ampdu_tid[tid].ssv_baw_size = IEEE80211_MIN_AMPDU_BUF << sta->ht_cap.ampdu_factor;

    if(buffer_size > IEEE80211_MAX_AMPDU_BUF)
    {
        buffer_size = IEEE80211_MAX_AMPDU_BUF;
    }

    if(ssv_sta_priv->ampdu_tid[tid].ssv_baw_size > buffer_size)
    {
        ssv_sta_priv->ampdu_tid[tid].ssv_baw_size = buffer_size;
    }
#endif

    ssv_sta_priv->ampdu_tid[tid].state = AMPDU_STATE_OPERATION;
} // end of - ssv6200_ampdu_tx_operation -

static void _clear_mpdu_q (struct ieee80211_hw *hw, struct sk_buff_head *q,
                           bool aggregated_mpdu)
{
    struct sk_buff *skb;
    while (1)
    {
        skb = skb_dequeue(q);
        if (!skb)
            break;
        // Remove AMPDU delimiter if it has been aggregation prepared.
        if (aggregated_mpdu)
            skb_pull(skb, AMPDU_DELIMITER_LEN);
        //ieee80211_tx_status_irqsafe(hw, skb);
        ieee80211_tx_status(hw, skb);
    }
} // end of - clear_mpdu_q -

// Freddie ToDo: In what condition the TX stop is called?
void ssv6200_ampdu_tx_stop (u16 tid, struct ieee80211_sta *sta,
                            struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;
    struct ssv_sta_priv_data *ssv_sta_priv;

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    ssv_sta_priv->ampdu_tid[tid].state = AMPDU_STATE_STOP;
    printk("ssv6200_ampdu_tx_stop\n");

    if (!list_empty(&sc->tx.ampdu_tx_que))
    {
#ifdef DEBUG_AMPDU_FLUSH
        {
            int j;
            struct AMPDU_TID_st *ampdu_tid = &ssv_sta_priv->ampdu_tid[tid];

            for (j = 0; j <= MAX_TID; j++)
            {
                if (sc->tid[j] == ampdu_tid)
                    break;
            }

            if (j == MAX_TID)
            {
                printk(KERN_ERR "No TID found when deleting it.\n");
            }
            else
                sc->tid[j] = NULL;

            printk(KERN_ERR "Deleting %02X-%02X-%02X-%02X-%02X-%02X TID %d (%p).\n",
                   sta->addr[0], sta->addr[1], sta->addr[2],
                   sta->addr[3], sta->addr[4], sta->addr[5],
                   ampdu_tid->tidno, ampdu_tid);
        }
#endif // DEBUG_AMPDU_FLUSH
        // del =====================================
        list_del_rcu(&ssv_sta_priv->ampdu_tid[tid].list);
    }
// Freddie ToDo: When AMPDU session stop, drop frame or put all of queued frames
// Frames in status array
// Frames in early AMPDU
// Frames in TID TX queue.

    printk("clear tx q len=%d\n",
           skb_queue_len(&ssv_sta_priv->ampdu_tid[tid].ampdu_skb_tx_queue));
    _clear_mpdu_q(sc->hw, &ssv_sta_priv->ampdu_tid[tid].ampdu_skb_tx_queue,
                  true);

    // Freddie ToDo: Clear skb at skb status array
    // skb queue clearance should be handled in TX thread. This callback 
    // function just wake it up and wait for clearance completed.

    printk("clear retry q len=%d\n",
           skb_queue_len(&ssv_sta_priv->ampdu_tid[tid].retry_queue));
    _clear_mpdu_q(sc->hw, &ssv_sta_priv->ampdu_tid[tid].retry_queue, true);

#ifdef USE_ENCRYPT_WORK
    printk("clear encrypt q len=%d\n",skb_queue_len(&ssv_sta_priv->ampdu_tid[tid].ampdu_skb_wait_encry_queue));
    _clear_mpdu_q(sc->hw, &ssv_sta_priv->ampdu_tid[tid].ampdu_skb_wait_encry_queue, false);
#endif // USE_ENCRYPT_WORK
#ifdef ENABLE_AGGREGATE_IN_TIME
    if (ssv_sta_priv->ampdu_tid[tid].cur_ampdu_pkt != NULL)
    {
        dev_kfree_skb_any(ssv_sta_priv->ampdu_tid[tid].cur_ampdu_pkt);
        ssv_sta_priv->ampdu_tid[tid].cur_ampdu_pkt = NULL;
    }
#endif // ENABLE_AGGREGATE_IN_TIME
    ssv6200_tx_flow_control((void *) sc,
                            sc->tx.hw_txqid[ssv_sta_priv->ampdu_tid[tid].ac],
                            false, 1000);
} // end of - ssv6200_ampdu_tx_stop -

static void ssv6200_ampdu_tx_state_stop_func (
        struct ssv_softc *sc, struct ieee80211_sta *sta, struct sk_buff *skb,
        struct AMPDU_TID_st *cur_AMPDU_TID)
{
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
    u8 *skb_qos_ctl = ieee80211_get_qos_ctl(hdr);
    u8 tid_no = skb_qos_ctl[0] & 0xf;

    // Check if ampdu_tx has to be enabled for this tid 
    if ((sta->ht_cap.ht_supported == true)
        && (!!(sc->sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_TX)))
    {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,32)
        ieee80211_start_tx_ba_session(sc->hw, (u8*)(sta->addr), (u16)tid_no);
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,37)
        ieee80211_start_tx_ba_session(sta, tid_no);
#else
        ieee80211_start_tx_ba_session(sta, tid_no, 0);
#endif
        ampdu_db_log("start ampdu_tx(rc) : tid_no = %d\n", tid_no);
    }
} // end of - ssv6200_ampdu_tx_state_stop_func -

static void ssv6200_ampdu_tx_state_operation_func (
        struct ssv_softc *sc, struct ieee80211_sta *sta, struct sk_buff *skb,
        struct AMPDU_TID_st *cur_AMPDU_TID)
{
    //struct sk_buff *ampdu_skb;
    //struct ieee80211_tx_info *info;
    //u32 quene_len, temp_i;
    //int ampdu_skb_tx_queue_lock_ret;
    //int ampdu_skb_tx_waitBA_queue_lock_ret;

#if 0
    else if (sc->ampdu_rekey_pause == AMPDU_REKEY_PAUSE_ONGOING)
    {
        pause_ampdu = true;
        printk("!!!ampdu_rekey_pause\n");
    }
#endif

}

void ssv6200_ampdu_tx_update_state (void *priv, struct ieee80211_sta *sta,
                                    struct sk_buff *skb)
{
    struct ssv_softc *sc = (struct ssv_softc *) priv;
    struct ssv_sta_priv_data *ssv_sta_priv =
            (struct ssv_sta_priv_data *) sta->drv_priv;
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
    u8 *skb_qos_ctl;
    u8 tid_no;

    {
        skb_qos_ctl = ieee80211_get_qos_ctl(hdr);
        tid_no = skb_qos_ctl[0] & 0xf;

        switch (ssv_sta_priv->ampdu_tid[tid_no].state)
        {
            case AMPDU_STATE_STOP:
                ssv6200_ampdu_tx_state_stop_func(
                        sc, sta, skb, &(ssv_sta_priv->ampdu_tid[tid_no]));
                break;

            case AMPDU_STATE_START:
                break;

            case AMPDU_STATE_OPERATION:
                ssv6200_ampdu_tx_state_operation_func(
                        sc, sta, skb, &(ssv_sta_priv->ampdu_tid[tid_no]));
                break;

            default:
                break;
        }
    }
}

void _put_mpdu_to_ampdu (struct sk_buff *ampdu, struct sk_buff *mpdu)
{
    bool is_empty_ampdu = (ampdu->len == 0);
    unsigned char *data_dest;
    struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *) ampdu->head;

    BUG_ON(skb_tailroom(ampdu) < mpdu->len);

    data_dest = skb_tail_pointer(ampdu);

    skb_put(ampdu, mpdu->len);

    // Copy TX info and queue mapping of first MPDU to AMPDU.
    if (is_empty_ampdu)
    {
        struct ieee80211_tx_info *ampdu_info = IEEE80211_SKB_CB(ampdu);
        struct ieee80211_tx_info *mpdu_info = IEEE80211_SKB_CB(mpdu);
        SKB_info                 *mpdu_skb_info = (SKB_info *)(mpdu->head);
        u32                       max_size_for_rate = ampdu_max_transmit_length[mpdu_skb_info->lowest_rate];

        BUG_ON(max_size_for_rate == 0);

        memcpy(ampdu_info, mpdu_info, sizeof(struct ieee80211_tx_info));

        skb_set_queue_mapping(ampdu, skb_get_queue_mapping(mpdu));
        ampdu_hdr->first_sn = ampdu_skb_ssn(mpdu);
        ampdu_hdr->sta = ((struct SKB_info_st *)mpdu->head)->sta;

        // Get maximum aggregation length according to the rate of the first MPDU and
        if (ampdu_hdr->max_size > max_size_for_rate)
            ampdu_hdr->max_size = max_size_for_rate;

        memcpy(ampdu_hdr->rates, mpdu_skb_info->rates, sizeof(ampdu_hdr->rates));
    }

    memcpy(data_dest, mpdu->data, mpdu->len);
    __skb_queue_tail(&ampdu_hdr->mpdu_q, mpdu);
    ampdu_hdr->ssn[ampdu_hdr->mpdu_num++] = ampdu_skb_ssn(mpdu);
    ampdu_hdr->size += mpdu->len;
    BUG_ON(ampdu_hdr->size > ampdu_hdr->max_size);
} // end of - _put_mpdu_to_ampdu -

u32 _flush_early_ampdu_q (struct ssv_softc *sc, struct AMPDU_TID_st *ampdu_tid)
{
    u32 flushed_ampdu = 0;
    unsigned long flags;
    struct sk_buff_head *early_aggr_ampdu_q = &ampdu_tid->early_aggr_ampdu_q;

    // Lock early AMPDU queue because both RX task and TX task would call this function.
    spin_lock_irqsave(&early_aggr_ampdu_q->lock, flags);

    // Check room in BA window. If there is enough space for one AMPDU,
    // put head of aggregated AMPDU to HCI queue.
    while (skb_queue_len(early_aggr_ampdu_q))
    {
        struct sk_buff *head_ampdu;
        struct ampdu_hdr_st *head_ampdu_hdr;
        u32 ampdu_aggr_num;

        head_ampdu = skb_peek(early_aggr_ampdu_q);
        head_ampdu_hdr = (struct ampdu_hdr_st *) head_ampdu->head;
        ampdu_aggr_num = skb_queue_len(&head_ampdu_hdr->mpdu_q);

        if ((SSV_AMPDU_BA_WINDOW_SIZE - ampdu_tid->aggr_pkt_num)
            < ampdu_aggr_num)
            break;

        //   Put to HCI queue.
        if (_sync_ampdu_pkt_arr(ampdu_tid, head_ampdu, false))
        {
            head_ampdu = __skb_dequeue(early_aggr_ampdu_q);
            ampdu_tid->early_aggr_skb_num -= ampdu_aggr_num;
            // Disable flow control if skb number in early aggregation queue is less than flow control lower bound.
#ifdef SSV_AMPDU_FLOW_CONTROL
            if (ampdu_tid->early_aggr_skb_num
                <= SSV_AMPDU_FLOW_CONTROL_LOWER_BOUND)
            {
                ssv6200_tx_flow_control((void *) sc,
                                        sc->tx.hw_txqid[ampdu_tid->ac], false, 1000);
            }
#endif // SSV_AMPDU_FLOW_CONTROL
            // Debug code
            if ((skb_queue_len(early_aggr_ampdu_q) == 0)
                && (ampdu_tid->early_aggr_skb_num > 0))
            {
                printk(KERN_ERR "Empty early Q w. %d.\n", ampdu_tid->early_aggr_skb_num);
            }
            _send_hci_skb(sc, head_ampdu,
                          AMPDU_HCI_SEND_TAIL_WITHOUT_FLOWCTRL);
            flushed_ampdu++;
        }
        else
            break;
    }
    spin_unlock_irqrestore(&early_aggr_ampdu_q->lock, flags);

    return flushed_ampdu;
} // end of - _flush_early_ampdu_q -


// Incremental aggregate AMPDU frame
void _aggr_ampdu_tx_q (struct ieee80211_hw *hw, struct AMPDU_TID_st *ampdu_tid)
{
    struct ssv_softc *sc = hw->priv;
    struct sk_buff *ampdu_skb = ampdu_tid->cur_ampdu_pkt;

    // Aggregate prepared frame into AMPDU.
    while (skb_queue_len(&ampdu_tid->ampdu_skb_tx_queue))
    {
        u32 aggr_len;
        struct sk_buff *mpdu_skb;
        struct ampdu_hdr_st *ampdu_hdr;
        bool is_aggr_full = false;
        //
        if (ampdu_skb == NULL)
        {
            ampdu_skb = _alloc_ampdu_skb(sc, ampdu_tid, 0);
            if (ampdu_skb == NULL)
                break;
            ampdu_tid->cur_ampdu_pkt = ampdu_skb;
        }

        // Fill up AMPDU with queued aggregation-ready MPDUs.
        ampdu_hdr = (struct ampdu_hdr_st *) ampdu_skb->head;
        aggr_len = skb_queue_len(&ampdu_hdr->mpdu_q);

        //BUG_ON(aggr_len >= ampdu_tid->agg_num_max);

        do
        {
            struct sk_buff_head *tx_q = &ampdu_tid->ampdu_skb_tx_queue;
            unsigned long flags;

            spin_lock_irqsave(&tx_q->lock, flags);
            //mpdu_skb = skb_dequeue(&ampdu_tid->ampdu_skb_tx_queue);
            mpdu_skb = skb_peek(&ampdu_tid->ampdu_skb_tx_queue);
            if (mpdu_skb == NULL)
            {
                spin_unlock_irqrestore(&tx_q->lock, flags);
                break;
            }
            if ((mpdu_skb->len + ampdu_hdr->size) > ampdu_hdr->max_size)
            {
                is_aggr_full = true;
                spin_unlock_irqrestore(&tx_q->lock, flags);
                break;
            }
            mpdu_skb = __skb_dequeue(&ampdu_tid->ampdu_skb_tx_queue);
            spin_unlock_irqrestore(&tx_q->lock, flags);

            _put_mpdu_to_ampdu(ampdu_skb, mpdu_skb);
        } while (++aggr_len < MAX_AGGR_NUM /*ampdu_tid->agg_num_max*/);

        // Enough MPDU # in AMPDU or both BA window and HCI queue are empty.
        if (   (is_aggr_full || (aggr_len >= MAX_AGGR_NUM)) // (aggr_len == ampdu_tid->agg_num_max)
            || (   (aggr_len > 0)
                && (skb_queue_len(&ampdu_tid->early_aggr_ampdu_q) == 0)
                && (ampdu_tid->ssv_baw_head == SSV_ILLEGAL_SN) // BA window empty
                && _is_skb_q_empty(sc, ampdu_skb))) // Corresponding HCI Q is empty
        {
            _add_ampdu_txinfo(sc, ampdu_skb);

            // Put to AMPDU queue.
            _queue_early_ampdu(sc, ampdu_tid, ampdu_skb);
            ampdu_tid->cur_ampdu_pkt = ampdu_skb = NULL;
        }

        _flush_early_ampdu_q(sc, ampdu_tid);
    }
} // end of - _aggr_ampdu_tx_q -


void _queue_early_ampdu (struct ssv_softc *sc, struct AMPDU_TID_st *ampdu_tid,
                         struct sk_buff *ampdu_skb)
{
    unsigned long flags;
    struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *) ampdu_skb->head;

    spin_lock_irqsave(&ampdu_tid->early_aggr_ampdu_q.lock, flags);
    __skb_queue_tail(&ampdu_tid->early_aggr_ampdu_q, ampdu_skb);
    ampdu_tid->early_aggr_skb_num += skb_queue_len(&ampdu_hdr->mpdu_q);
    // Enable flow control if skb num in early aggregation queue is greater than flow control lower bound.
#ifdef SSV_AMPDU_FLOW_CONTROL
    if (ampdu_tid->early_aggr_skb_num >= SSV_AMPDU_FLOW_CONTROL_UPPER_BOUND)
    {
        ssv6200_tx_flow_control((void *) sc, sc->tx.hw_txqid[ampdu_tid->ac],
                                true, 1000);
    }
#endif // SSV_AMPDU_FLOW_CONTROL
    spin_unlock_irqrestore(&ampdu_tid->early_aggr_ampdu_q.lock, flags);
} // end of - _queue_early_ampdu -


void _flush_mpdu (struct ssv_softc *sc, struct ieee80211_sta *sta)
{
    unsigned long flags;
    struct ssv_sta_priv_data *ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    int i;

    // Flush MPDU in early aggregated AMPDU
    for (i = 0; i < (sizeof(ssv_sta_priv->ampdu_tid) / sizeof(ssv_sta_priv->ampdu_tid[0])); i++)
    {
        struct AMPDU_TID_st *ampdu_tid = &ssv_sta_priv->ampdu_tid[i];
        struct sk_buff_head *early_aggr_ampdu_q;
        struct sk_buff      *ampdu;
        struct ampdu_hdr_st *ampdu_hdr;
        struct sk_buff_head *mpdu_q;
        struct sk_buff      *mpdu;

        if (ampdu_tid->state != AMPDU_STATE_OPERATION)
            continue;

        early_aggr_ampdu_q = &ampdu_tid->early_aggr_ampdu_q;

        // Lock early AMPDU queue because both RX task and TX task would call this function.
        spin_lock_irqsave(&early_aggr_ampdu_q->lock, flags);

        // Flush MPDUs of each AMPDU in early aggregation queue.
        while ((ampdu = __skb_dequeue(early_aggr_ampdu_q)) != NULL)
        {
            ampdu_hdr = (struct ampdu_hdr_st *)ampdu->head;
            mpdu_q = &ampdu_hdr->mpdu_q;

            while ((mpdu = __skb_dequeue(mpdu_q)) != NULL)
            {
                _send_hci_skb(sc, mpdu, AMPDU_HCI_SEND_TAIL_WITHOUT_FLOWCTRL);
            }
            ssv6200_ampdu_release_skb(ampdu, sc->hw);
        }

        // Flush MPDU in current aggregating AMPDU.
        if (ampdu_tid->cur_ampdu_pkt != NULL)
        {
            ampdu_hdr = (struct ampdu_hdr_st *)ampdu_tid->cur_ampdu_pkt->head;
            mpdu_q = &ampdu_hdr->mpdu_q;
            while ((mpdu = __skb_dequeue(mpdu_q)) != NULL)
            {
                _send_hci_skb(sc, mpdu, AMPDU_HCI_SEND_TAIL_WITHOUT_FLOWCTRL);
            }
            ssv6200_ampdu_release_skb(ampdu_tid->cur_ampdu_pkt, sc->hw);
            ampdu_tid->cur_ampdu_pkt = NULL;
        }

        spin_unlock_irqrestore(&early_aggr_ampdu_q->lock, flags);
    }
} // end of - _flush_mpdu -

bool ssv6200_ampdu_tx_handler (struct ieee80211_hw *hw, struct sk_buff *skb)
{
    struct ssv_softc           *sc = hw->priv;
    struct ieee80211_hdr       *hdr = (struct ieee80211_hdr *) skb->data;
//    struct ieee80211_tx_info   *info = IEEE80211_SKB_CB(skb);
    struct SKB_info_st         *mpdu_skb_info_p = (SKB_info *) (skb->head);
    struct ieee80211_sta       *sta = mpdu_skb_info_p->sta;
    struct ssv_sta_priv_data   *ssv_sta_priv =
            (struct ssv_sta_priv_data *) sta->drv_priv;
    u8                          tidno;
    struct AMPDU_TID_st        *ampdu_tid;

    if (sta == NULL)
    {
        WARN_ON(1);
        return false;
    }
    tidno = ieee80211_get_qos_ctl(hdr)[0] & IEEE80211_QOS_CTL_TID_MASK;
    ampdu_db_log("tidno = %d\n", tidno);

    ampdu_tid = &ssv_sta_priv->ampdu_tid[tidno];
    // Process packet only when AMPDU of its TID is in operation.
    if (ampdu_tid->state != AMPDU_STATE_OPERATION)
        return false;

#ifdef AMPDU_CHECK_SKB_SEQNO
    {
        u32 skb_seqno = ((struct ieee80211_hdr*) (skb->data))->seq_ctrl
                        >> SSV_SEQ_NUM_SHIFT;
        u32 tid_seqno = ampdu_tid->last_seqno;
        //if ((skb_seqno % 256) == 0)
        //    prn_aggr_err("%d - %d\n",  tid_seqno, skb_seqno);
        if ((tid_seqno != (-1)) && (skb_seqno != NEXT_PKT_SN(tid_seqno)))
        {
            prn_aggr_err("Non continueous seq no: %d - %d\n", tid_seqno, skb_seqno);
            return false;
        }    
        ampdu_tid->last_seqno = skb_seqno;
    }
#endif // AMPDU_CHECK_SKB_SEQNO

#if 1
    // Get maximum aggregation length according to the rate of the first MPDU and
    mpdu_skb_info_p->lowest_rate = ssv62xx_ht_rate_update(skb, sc, mpdu_skb_info_p->rates);
    // Check if aggregation can be applied to current rate.
    if (ampdu_max_transmit_length[mpdu_skb_info_p->lowest_rate] == 0)
    {
        _flush_mpdu(sc, sta);
        return false;
    }
#endif

#if 1
    // Just by-pass it if both HCI Q and AMPDU TX Q are empty,
    if ((ampdu_tid->ssv_baw_head == SSV_ILLEGAL_SN) // No skb in BA window
        && (skb_queue_len(&ampdu_tid->ampdu_skb_tx_queue) == 0) // No skb is waiting in queue for aggregation.
        && (skb_queue_len(&ampdu_tid->early_aggr_ampdu_q) == 0) // No early aggregated AMPDU
        && ((ampdu_tid->cur_ampdu_pkt == NULL) // No partial early aggregated AMPDU
            || (skb_queue_len(
                    &((struct ampdu_hdr_st *) (ampdu_tid->cur_ampdu_pkt->head))->mpdu_q)
                == 0))
        && _is_skb_q_empty(sc, skb)) // Corresponding HCI queue is empty.
    {
        ampdu_tid->mib.ampdu_mib_pass_counter++;
#if 0
        prn_aggr_err(
                "pass %d\n",
                ((struct ieee80211_hdr*)(skb->data))->seq_ctrl >> SSV_SEQ_NUM_SHIFT);
#else
        if ((ssv_sta_priv->ampdu_tid[tidno].mib.ampdu_mib_pass_counter % 1024) == 0)
            prn_aggr_err("STA %d TID %d passed %d\n", ssv_sta_priv->sta_idx,
                         ampdu_tid->tidno,
                         ampdu_tid->mib.ampdu_mib_pass_counter);
#endif
        return false;
    }
#endif // 0
    // init mpdu_skb_info
    mpdu_skb_info_p = (SKB_info *) (skb->head);
    mpdu_skb_info_p->mpdu_retry_counter = 0;
    //mpdu_skb_info_p->mpdu_group_id = 0;
    mpdu_skb_info_p->ampdu_tx_status = AMPDU_ST_NON_AMPDU;
    mpdu_skb_info_p->ampdu_tx_final_retry_count = 0;

    ssv_sta_priv->ampdu_tid[tidno].ac = skb_get_queue_mapping(skb);
#if 0
#ifndef CONFIG_SSV_SW_ENCRYPT_HW_DECRYPT
    // Freddie ToDo: Security type is STA-based. Should check against STA configuration.
    if ((sc->ampdu_ccmp_encrypt == true))
    {
        //u32 skb_seqno = (hdr->seq_ctrl>>SSV_SEQ_NUM_SHIFT);
        //printk(KERN_ALERT "%d - tx skb_seqno=%d\n",  tidno, skb_seqno);        
        skb_queue_tail(&ssv_sta_priv->ampdu_tid[tidno].ampdu_skb_wait_encry_queue, skb);
        if (sc->ampdu_encry_work_scheduled == false)
        {
            schedule_work(&sc->ampdu_tx_encry_work);
            //schedule_work_on(3,&sc->ampdu_tx_encry_work);
            //sc->ampdu_encry_work_scheduled = true;
        }
    }
    else if (sc->algorithm != ME_NONE)
    {
        printk("None ccmp skb into AMPDU sc->algorithm = %d\n",sc->algorithm);

        info = IEEE80211_SKB_CB(skb);
        info->flags &= (~IEEE80211_TX_CTL_AMPDU);

        if(ssv6200_mpdu_send_HCI(sc->hw, skb))
        ieee80211_tx_status_irqsafe(sc->hw, skb); //release skb when hci send fail.
    }
    else
#endif // CONFIG_SSV_SW_ENCRYPT_HW_DECRYPT
#endif // 0
    {
        bool ret;
        // Make it aggregation ready as an MPDU of AMPDU by adding delimiter, FCS, and padding.
        ret = ssv6200_ampdu_add_delimiter_and_crc32(skb);
        if (ret == false)
        {
            // Freddie ToDo: Report dropped TX skb?
            ssv6200_ampdu_release_skb(skb, hw);
            return false;
        }
        // Put it into AMPDU TX queue.
        skb_queue_tail(&ssv_sta_priv->ampdu_tid[tidno].ampdu_skb_tx_queue, skb);
        // Keep track time of the newest MPDU to the AMPDU session.
        ssv_sta_priv->ampdu_tid[tidno].timestamp = jiffies;
    }

    _aggr_ampdu_tx_q(hw, &ssv_sta_priv->ampdu_tid[tidno]);

    return true;
} // end of - ssv6200_ampdu_tx_handler -


u32 ssv6xxx_ampdu_flush (struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;
    struct AMPDU_TID_st *cur_AMPDU_TID;
    u32 flushed_ampdu = 0;
    u32 tid_idx = 0;

    if (!list_empty(&sc->tx.ampdu_tx_que))
    {
        list_for_each_entry_rcu(cur_AMPDU_TID, &sc->tx.ampdu_tx_que, list)
        {
            tid_idx++;

            //WARN_ON(cur_AMPDU_TID->state != AMPDU_STATE_OPERATION);
#ifdef DEBUG_AMPDU_FLUSH
            {
                int i = 0;
                for (i = 0; i < MAX_TID; i++)
                    if (sc->tid[i] == cur_AMPDU_TID)
                        break;
                if (i == MAX_TID)
                {
                    printk(KERN_ERR "No matching TID (%d) found! %p\n", tid_idx, cur_AMPDU_TID);
                    continue;
                }
            }
#endif // DEBUG_AMPDU_FLUSH
            if (cur_AMPDU_TID->state != AMPDU_STATE_OPERATION)
            {
                struct ieee80211_sta *sta = cur_AMPDU_TID->sta;
                struct ssv_sta_priv_data *sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;
                dev_dbg(sc->dev, "STA %d TID %d is @%d\n",
                        sta_priv->sta_idx, cur_AMPDU_TID->tidno,
                        cur_AMPDU_TID->state);
                continue;
            }

            // No buffered AMPDU? Send current aggregating one.
            if ((cur_AMPDU_TID->state == AMPDU_STATE_OPERATION)
                && (skb_queue_len(&cur_AMPDU_TID->early_aggr_ampdu_q) == 0)
                && (cur_AMPDU_TID->cur_ampdu_pkt != NULL))
            {
                struct ampdu_hdr_st *ampdu_hdr =
                        (struct ampdu_hdr_st *) (cur_AMPDU_TID->cur_ampdu_pkt->head);
                u32 aggr_len = skb_queue_len(&ampdu_hdr->mpdu_q);
                if (aggr_len)
                {
                    struct sk_buff *ampdu_skb = cur_AMPDU_TID->cur_ampdu_pkt;

                    cur_AMPDU_TID->cur_ampdu_pkt = NULL;

                    _add_ampdu_txinfo(sc, ampdu_skb);

                    // Put to AMPDU queue.
                    _queue_early_ampdu(sc, cur_AMPDU_TID, ampdu_skb);

                    #if 0
                    prn_aggr_err("A%c %d %d\n", sc->tx_q_empty ? 'e' : 't', 
                                ampdu_hdr->first_sn, aggr_len);
                    #endif
                }
            }
            if (skb_queue_len(&cur_AMPDU_TID->early_aggr_ampdu_q) > 0)
                flushed_ampdu += _flush_early_ampdu_q(sc, cur_AMPDU_TID);
        }
    }
    //spin_unlock(&sc->tx.ampdu_tx_que_lock);
    return flushed_ampdu;
} // end of - ssv6xxx_ampdu_flush -

int _dump_BA_notification (char *buf,
                            struct ampdu_ba_notify_data *ba_notification)
{
    int i;
    char *orig_buf = buf;

    for (i = 0; i < MAX_AGGR_NUM; i++)
    {
        if (ba_notification->seq_no[i] == (u16) (-1))
            break;
        buf += sprintf(buf, " %d", ba_notification->seq_no[i]);
    }
    return ((size_t)buf - (size_t)orig_buf);
}


int _dump_ba_skb (char *buf, int buf_size, struct sk_buff *ba_skb)
{
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) (ba_skb->data
                                                          + SSV6XXX_RX_DESC_LEN);
    AMPDU_BLOCKACK *BA_frame = (AMPDU_BLOCKACK *) hdr;
    u32 ssn = BA_frame->BA_ssn;
    struct ampdu_ba_notify_data *ba_notification =
            (struct ampdu_ba_notify_data *) (ba_skb->data + ba_skb->len
                                             - sizeof(struct ampdu_ba_notify_data));
    int prt_size;
    
    prt_size = snprintf(buf, buf_size, "\n\t\t%04d %08X %08X -", 
                        ssn, BA_frame->BA_sn_bit_map[0], BA_frame->BA_sn_bit_map[1]);
    buf_size -= prt_size;
    buf += prt_size;
    
    prt_size = prt_size + _dump_BA_notification(buf, ba_notification);
    return prt_size;
}


static bool _ssn_to_bit_idx (u32 start_ssn, u32 mpdu_ssn, u32 *word_idx,
                             u32 *bit_idx)
{
    u32 ret_bit_idx, ret_word_idx = 0;
    s32 diff = mpdu_ssn - start_ssn;

    if (diff >= 0)
    {
        if (diff >= SSV_AMPDU_BA_WINDOW_SIZE)
        {
            //printk(KERN_ERR "O %d %d\n", start_ssn, mpdu_ssn);
            return false;
        }
        ret_bit_idx = diff;
    }
    else
    {
        diff = -diff;
        if (diff <= (SSV_AMPDU_MAX_SSN - SSV_AMPDU_BA_WINDOW_SIZE))
        {
            // Just to suppress compiler warning about uninitialized variable.
            //printk(KERN_ERR "O %d %d\n", start_ssn, mpdu_ssn);
            *word_idx = 0;
            *bit_idx = 0;
            return false;
        }
        ret_bit_idx = SSV_AMPDU_MAX_SSN - diff;
    }

    if (ret_bit_idx >= 32)
    {
        ret_bit_idx -= 32;
        ret_word_idx = 1;
    }

    *bit_idx = ret_bit_idx;
    *word_idx = ret_word_idx;
    return true;
} // end of - _ssn_to_bit_idx -

static bool _inc_bit_idx (u32 ssn_1st, u32 ssn_next, u32 *word_idx,
                          u32 *bit_idx)
{
    u32 ret_word_idx = *word_idx, ret_bit_idx = *bit_idx;
    s32 diff = (s32) ssn_1st - (s32) ssn_next;

    if (diff > 0)
    {
        if (diff < (SSV_AMPDU_MAX_SSN - SSV_AMPDU_BA_WINDOW_SIZE))
        {
            prn_aggr_err("Irrational SN distance in AMPDU: %d %d.\n",
                         ssn_1st, ssn_next);
            return false;
        }
        diff = SSV_AMPDU_MAX_SSN - diff;
    }
    else
    {
        diff = -diff;
    }

    // Debug: distance between two aggregated MPDU should not greater than BA window.
    if (diff > SSV_AMPDU_MAX_SSN)
        prn_aggr_err("DF %d - %d = %d\n", ssn_1st, ssn_next, diff);

    ret_bit_idx += diff;

    if (ret_bit_idx >= 32)
    {
        ret_bit_idx -= 32;
        ret_word_idx++;
    }

    *word_idx = ret_word_idx;
    *bit_idx = ret_bit_idx;
    return true;
} // end of - _inc_bit_idx -

static void _release_frames (struct AMPDU_TID_st *ampdu_tid)
{
    u32 head_ssn, head_ssn_before, last_ssn;
    struct sk_buff **skb;
    struct SKB_info_st *skb_info;
    //unsigned long flags;

    //spin_lock_irqsave(&ampdu_tid->pkt_array_lock, flags);
    spin_lock_bh(&ampdu_tid->pkt_array_lock);

    head_ssn_before = ampdu_tid->ssv_baw_head;
#if 1
    if (head_ssn_before >= SSV_AMPDU_MAX_SSN)
    {
        //spin_unlock_irqrestore(&ampdu_tid->pkt_array_lock, flags);
        spin_unlock_bh(&ampdu_tid->pkt_array_lock);
        prn_aggr_err("l x.x %d\n", head_ssn_before);
        return;
    }
#else
    BUG_ON(head_ssn_before >= SSV_AMPDU_MAX_SSN);
#endif // 0
    head_ssn = ampdu_tid->ssv_baw_head;
    last_ssn = head_ssn;

    do
    {
        skb = &INDEX_PKT_BY_SSN(ampdu_tid, head_ssn);
        if (*skb == NULL)
        {
            // All packet are released.
            head_ssn = SSV_ILLEGAL_SN;
            // Debug code
            //if (ampdu_tid->aggr_pkt_num)
            {
                int i;
                char sn_str[66 * 5] = "";
                char *str = sn_str;
                for (i = 0; i < 64; i++)
                    if (ampdu_tid->aggr_pkts[i] != NULL)
                    {
                        str += sprintf(str, "%d ",
                                       ampdu_skb_ssn(ampdu_tid->aggr_pkts[i]));
                    }
                *str = 0;
                if (str == sn_str)
                {
                    // prn_aggr_err("ILL %d %d - %d\n", head_ssn_before, last_ssn, ampdu_tid->aggr_pkt_num);
                }
                else
                    prn_aggr_err(
                            "ILL %d %d - %d (%s)\n",
                            head_ssn_before, last_ssn, ampdu_tid->aggr_pkt_num, sn_str);
            }
            break;
        }
        skb_info = (struct SKB_info_st *) ((*skb)->head);
        if ((skb_info->ampdu_tx_status == AMPDU_ST_DONE)
            || (skb_info->ampdu_tx_status == AMPDU_ST_DROPPED))
        {
            __skb_queue_tail(&ampdu_tid->release_queue, *skb);
            //prn_aggr_err("%c %d %d\n", (skb_info->ampdu_tx_status == AMPDU_ST_DONE) ? 'D' : 'P', head_ssn, ampdu_skb_ssn(*skb));
            *skb = NULL;
            last_ssn = head_ssn;
            INC_PKT_SN(head_ssn);
            ampdu_tid->aggr_pkt_num--;
            if (skb_info->ampdu_tx_status == AMPDU_ST_DROPPED)
                ampdu_tid->mib.ampdu_mib_discard_counter++;
        }
        else
        {
            break;
        }
    } while (1);

    ampdu_tid->ssv_baw_head = head_ssn;

#if 0
    if (head_ssn == SSV_ILLEGAL_SN)
    {
        u32 i = head_ssn_before;
        do
        {
            skb = &INDEX_PKT_BY_SSN(ampdu_tid, i);
            if (*skb != NULL)
            prn_aggr_err("O.o %d: %d - %d\n", head_ssn_before, i, ampdu_skb_ssn(*skb));
            INC_PKT_SN(i);
        }while (i != head_ssn_before);
    }
#endif // 0
    //spin_unlock_irqrestore(&ampdu_tid->pkt_array_lock, flags);
    spin_unlock_bh(&ampdu_tid->pkt_array_lock);

#if 0
    if (head_ssn_before != head_ssn)
    {
        prn_aggr_err("H %d -> %d (%d - %d)\n", head_ssn_before, head_ssn,
                ampdu_tid->aggr_pkt_num, skb_queue_len(&ampdu_tid->ampdu_skb_tx_queue));
    }
#endif // 0
} // end of - _release_frames -


static int _collect_retry_frames (struct AMPDU_TID_st *ampdu_tid)
{
    u32             ssn, end_ssn;
    int             num_retry = 0;
    int             timeout_check = 1; // Check timeout for leading aggregated skb.
    unsigned long   check_jiffies = jiffies;

    ssn = ampdu_tid->ssv_baw_head;

    if (ssn == SSV_ILLEGAL_SN)
        return 0;

    end_ssn =   (ampdu_tid->ssv_baw_head + SSV_AMPDU_BA_WINDOW_SIZE)
              % SSV_AMPDU_MAX_SSN;

    do
    {
        struct sk_buff     *skb = INDEX_PKT_BY_SSN(ampdu_tid, ssn);
        struct SKB_info_st *skb_info;
        int                 timeout_retry = 0;

        if (skb == NULL)
            break;

        skb_info = (SKB_info *) (skb->head);

        if (   timeout_check
            && (skb_info->ampdu_tx_status == AMPDU_ST_SENT))
        {
            unsigned long cur_jiffies = jiffies;
            unsigned long timeout_jiffies =   skb_info->aggr_timestamp
                                            + msecs_to_jiffies(BA_WAIT_TIMEOUT);
            u32           delta_ms;

            if (time_before(cur_jiffies, timeout_jiffies))
            {
                timeout_check = 0;
                continue;
            }

            _mark_skb_retry(skb_info, skb);
            delta_ms = jiffies_to_msecs(cur_jiffies - skb_info->aggr_timestamp);
            prn_aggr_err("t S%d-T%d-%d (%u)\n",
                         ((struct ssv_sta_priv_data *)skb_info->sta->drv_priv)->sta_idx,
                         ampdu_tid->tidno, ssn,
                         delta_ms);
            if (delta_ms > 1000)
            {
                prn_aggr_err("Last checktime %lu - %lu = %u\n",
                             check_jiffies, ampdu_tid->timestamp,
                             jiffies_to_msecs(check_jiffies - ampdu_tid->timestamp));
            }
            timeout_retry = 1;
        }

        if (skb_info->ampdu_tx_status == AMPDU_ST_RETRY)
        {
#if 0
            if (!timeout_retry)
                prn_aggr_dbg("r %d - %d\n", ssn, ampdu_skb_ssn(skb));
#endif
            skb_queue_tail(&ampdu_tid->retry_queue, skb);
            ampdu_tid->mib.ampdu_mib_retry_counter++;
            num_retry++;
        }

        INC_PKT_SN(ssn);
    } while (ssn != end_ssn);

    ampdu_tid->timestamp = check_jiffies;
    return num_retry;
} // end of - _collect_retry_frames -

int _mark_skb_retry (struct SKB_info_st *skb_info, struct sk_buff *skb)
{
    if (skb_info->mpdu_retry_counter < SSV_AMPDU_retry_counter_max)
    {
        if (skb_info->mpdu_retry_counter == 0)
        {
            struct ieee80211_hdr *skb_hdr = ampdu_skb_hdr(skb);
            skb_hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_RETRY);
        }
        skb_info->ampdu_tx_status = AMPDU_ST_RETRY;
        //skb_queue_tail(&ampdu_tid->retry_queue, skb);
        skb_info->mpdu_retry_counter++;
        //prn_aggr_err("r %d\n", skb_ssn);
        return 1;
    }
    else
    {
        skb_info->ampdu_tx_status = AMPDU_ST_DROPPED;
        prn_aggr_err("p %d\n", ampdu_skb_ssn(skb));
        return 0;
    }
} // end of - _mark_skb_retry -


static u32 _ba_map_walker (struct AMPDU_TID_st *ampdu_tid, u32 start_ssn,
                           u32 sn_bit_map[2],
                           struct ampdu_ba_notify_data *ba_notify_data,
                           u32 *p_acked_num)
{
    int i = 0;
    u32 ssn = ba_notify_data->seq_no[0];
    u32 word_idx = (-1), bit_idx = (-1);
    bool found = _ssn_to_bit_idx(start_ssn, ssn, &word_idx, &bit_idx);
    bool first_found = found;
    u32 aggr_num = 0;
    u32 acked_num = 0;

    // Debug code
    if (found && (word_idx >= 2 || bit_idx >= 32))
        prn_aggr_err("idx error 1: %d %d %d %d\n",
                     start_ssn, ssn, word_idx, bit_idx);

    while ((i < MAX_AGGR_NUM) && (ssn < SSV_AMPDU_MAX_SSN))
    {
        u32 cur_ssn;
        struct sk_buff *skb = INDEX_PKT_BY_SSN(ampdu_tid, ssn);
        u32 skb_ssn = (skb == NULL) ? (-1) : ampdu_skb_ssn(skb);
        struct SKB_info_st *skb_info;

        aggr_num++;

        if (skb_ssn != ssn)
        {
            prn_aggr_err("Unmatched SSN packet: %d - %d - %d\n",
                         ssn, skb_ssn, start_ssn);
        }
        else
        {
            skb_info = (struct SKB_info_st *) (skb->head);

            if (found && (sn_bit_map[word_idx] & (1 << bit_idx)))
            {
                skb_info->ampdu_tx_status = AMPDU_ST_DONE;
                acked_num++;
            }
            else
            {
                _mark_skb_retry(skb_info, skb);
            }
        }

        cur_ssn = ssn;

        if (++i >= MAX_AGGR_NUM)
            break;

        ssn = ba_notify_data->seq_no[i];
        if (ssn >= SSV_AMPDU_MAX_SSN)
            break;

        if (first_found)
        {
            u32 old_word_idx = word_idx, old_bit_idx = bit_idx;
            found = _inc_bit_idx(cur_ssn, ssn, &word_idx, &bit_idx);
            if (found && (word_idx >= 2 || bit_idx >= 32))
            {
                prn_aggr_err(
                        "idx error 2: %d 0x%08X 0X%08X %d %d (%d %d) (%d %d)\n",
                        start_ssn, sn_bit_map[1], sn_bit_map[0], cur_ssn, ssn, word_idx, bit_idx, old_word_idx, old_bit_idx);
                found = false;
            }
            else if (!found)
            {
                char strbuf[256];
                _dump_BA_notification(strbuf, ba_notify_data);
                prn_aggr_err("SN out-of-order: %d\n%s\n", start_ssn, strbuf);
            }
        }
        else
        {
            found = _ssn_to_bit_idx(start_ssn, ssn, &word_idx, &bit_idx);
            first_found = found;
            if (found && (word_idx >= 2 || bit_idx >= 32))
                prn_aggr_err("idx error 3: %d %d %d %d\n",
                             cur_ssn, ssn, word_idx, bit_idx);
        }
    }
    // BA window head is ack-ed. Release leading ack-ed MPDUs in window.
    //prn_aggr_err("Br %d\n", start_ssn);
    _release_frames(ampdu_tid);

    if (p_acked_num != NULL)
        *p_acked_num = acked_num;
    return aggr_num;
} // end of - _ba_map_walker -

static void _flush_release_queue (struct ieee80211_hw *hw,
                                  struct sk_buff_head *release_queue)
{
    do
    {
        struct sk_buff *ampdu_skb = __skb_dequeue(release_queue);
        struct ieee80211_tx_info *tx_info;
        struct SKB_info_st *skb_info;

        if (ampdu_skb == NULL)
            break;

        //prn_aggr_err("F %d\n", ampdu_skb_ssn(ampdu_skb));

        skb_info = (struct SKB_info_st *) (ampdu_skb->head);

        skb_pull(ampdu_skb, AMPDU_DELIMITER_LEN); // remove AMPDU delimiter

        tx_info = IEEE80211_SKB_CB(ampdu_skb);
        ieee80211_tx_info_clear_status(tx_info);
        tx_info->flags |= IEEE80211_TX_STAT_AMPDU;
        if (skb_info->ampdu_tx_status == AMPDU_ST_DONE)
            tx_info->flags |= IEEE80211_TX_STAT_ACK;
        // ToDo: NO_BACK would make mac80211 send BAR.
        // tx_info->flags |= IEEE80211_TX_STAT_AMPDU_NO_BACK;

        // ToDo: report correct ampdu_len and ampdu_ack_len
        tx_info->status.ampdu_len = 1; // number of aggregated frames.
        tx_info->status.ampdu_ack_len = 1; // number of acked aggregated frames.

        // ToDo: Check if _irqsave should be used.
#if defined(USE_THREAD_RX) && !defined(IRQ_PROC_TX_DATA)
        //#if 0
        ieee80211_tx_status(hw, ampdu_skb);
#else
        ieee80211_tx_status_irqsafe(hw, ampdu_skb);
#endif // USE_THREAD_RX
    } while (1);
} // end of - _flush_release_queue -

#if 0
/*
 Placement analysis.
 */
static u16 _get_BA_notification_hits(u16 ssn,u32 *bit_map,struct ampdu_ba_notify_data *ba_notification,u16 *max_continue_hits,u16 *aggr_num)
{
    int i;
    u16 hits=0,continue_hits=0;
    u64 bitMap=0;

    if(bit_map)
    bitMap = ((u64)bit_map[1]<<32) | (u64)(bit_map[0]);

    for (i = 0; i < MAX_AGGR_NUM; i++)
    {
        if (ba_notification->seq_no[i] == (u16)(-1))
        break;

        //while(ba_notification->seq_no[i] > ssn)
        if(ssn <= ba_notification->seq_no[i])
        {
            if((bitMap>>(ba_notification->seq_no[i]-ssn))&0x1)
            {
                hits++;
                continue_hits++;
                if(*max_continue_hits<=continue_hits)
                *max_continue_hits = continue_hits;
            }
            else
            {
                continue_hits=0;
            }
        }
    }
    *aggr_num = i;
    return hits;
}
#endif // 0
void ssv6200_ampdu_no_BA_handler (struct ieee80211_hw *hw, struct sk_buff *skb)
{
    struct cfg_host_event *host_event = (struct cfg_host_event *) skb->data;
    struct ampdu_ba_notify_data *ba_notification =
            (struct ampdu_ba_notify_data *) &host_event->dat[0];
    // The frame header comes after BA notification for reference to the corresponding AMPDU TID.
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) (ba_notification + 1);
    struct ssv_softc *sc = hw->priv;
    struct ieee80211_sta *sta = ssv6xxx_find_sta_by_addr(sc, hdr->addr1); // addr1 is the RA of the dropped AMPDU frame.
    u8 tidno = ieee80211_get_qos_ctl(hdr)[0] & IEEE80211_QOS_CTL_TID_MASK;
    struct ssv_sta_priv_data *ssv_sta_priv;
    char seq_str[256];
    struct AMPDU_TID_st *ampdu_tid;
    int i;
    u16 aggr_num = 0;
    struct firmware_rate_control_report_data *report_data;

    if (sta == NULL)
    {
        prn_aggr_err(
                "NO BA for %d to unmatched STA %02X-%02X-%02X-%02X-%02X-%02X: %s\n",
                tidno, hdr->addr1[0], hdr->addr1[1], hdr->addr1[2], hdr->addr1[3], hdr->addr1[4], hdr->addr1[5], seq_str);
        dev_kfree_skb_any(skb);
        return;
    }

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    _dump_BA_notification(seq_str, ba_notification);
    prn_aggr_err(
            "NO BA for %d to %02X-%02X-%02X-%02X-%02X-%02X: %s\n",
            tidno, sta->addr[0], sta->addr[1], sta->addr[2], sta->addr[3], sta->addr[4], sta->addr[5], seq_str);

    ampdu_tid = &ssv_sta_priv->ampdu_tid[tidno];

    if (ampdu_tid->state != AMPDU_STATE_OPERATION)
    {
        dev_kfree_skb_any(skb);
        return;
    }

    // Go through BA notification and mark corresponding skb as retry.
    for (i = 0; i < MAX_AGGR_NUM; i++)
    {
        u32 ssn = ba_notification->seq_no[i];
        struct sk_buff *skb;
        u32 skb_ssn;
        struct SKB_info_st *skb_info;
        // End of packet in BA notification
        if (ssn >= (4096))
            break;

        aggr_num++;
        skb = INDEX_PKT_BY_SSN(ampdu_tid, ssn);
        // Verify packet SSN.
        skb_ssn = (skb == NULL) ? (-1) : ampdu_skb_ssn(skb);
        if (skb_ssn != ssn)
        {
            prn_aggr_err("Unmatched SSN packet: %d - %d\n", ssn, skb_ssn);
            continue;
        }

        skb_info = (struct SKB_info_st *) (skb->head);
        if (skb_info->ampdu_tx_status == AMPDU_ST_SENT)
        {
            if (skb_info->mpdu_retry_counter < SSV_AMPDU_retry_counter_max)
            {
                if (skb_info->mpdu_retry_counter == 0)
                {
                    struct ieee80211_hdr *skb_hdr = ampdu_skb_hdr(skb);
                    skb_hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_RETRY);
                }
                skb_info->ampdu_tx_status = AMPDU_ST_RETRY;
                //skb_queue_tail(&ampdu_tid->retry_queue, skb);
                skb_info->mpdu_retry_counter++;
                //prn_aggr_err("r %d\n", skb_ssn);
            }
            else
            {
                skb_info->ampdu_tx_status = AMPDU_ST_DROPPED;
                prn_aggr_err("p %d\n", skb_ssn);
            }
        }
        else
        {
            // skb in abnormal status. Should be AMPDU_ST_AGGREGATED after AMPDU sent to HCI queue.
            prn_aggr_err("S %d %d\n", skb_ssn, skb_info->ampdu_tx_status);
        }
    }
    //prn_aggr_err("Bn %d\n", ba_notification->seq_no[0]);
    _release_frames(ampdu_tid);

    // Report AMPDU TX status to rate control by reusing host event skb.
    host_event->h_event = SOC_EVT_RC_AMPDU_REPORT;

    report_data =
            (struct firmware_rate_control_report_data *) &host_event->dat[0];
    report_data->ampdu_len = aggr_num;
    report_data->ampdu_ack_len = 0;
    report_data->wsid =  ssv_sta_priv->sta_info->hw_wsid;

#if 0
    printk("AMPDU report NO BA!!wsid[%d]didx[%d]F[%d]R[%d]S[%d]\n",report_data->wsid,report_data->data_rate,report_data->mpduFrames,report_data->mpduFrameRetry,report_data->mpduFrameSuccess);
#endif

    skb_queue_tail(&sc->rc_report_queue, skb);
    if (sc->rc_sample_sechedule == 0)
        queue_work(sc->rc_sample_workqueue, &sc->rc_sample_work);
} // end of - ssv6200_ampdu_no_BA_handler -

void ssv6200_ampdu_BA_handler (struct ieee80211_hw *hw, struct sk_buff *skb)
{
    struct ssv_softc *sc = hw->priv;
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) (skb->data
                                                          + SSV6XXX_RX_DESC_LEN);
    AMPDU_BLOCKACK *BA_frame = (AMPDU_BLOCKACK *) hdr;
    struct ieee80211_sta *sta;
    struct ssv_sta_priv_data *ssv_sta_priv;
    struct ampdu_ba_notify_data *ba_notification;
    u32 ssn, aggr_num = 0, acked_num = 0;
    u8 tid_no;
    u32 sn_bit_map[2];
    struct firmware_rate_control_report_data *report_data;
    HDR_HostEvent *host_evt;

    sta = ssv6xxx_find_sta_by_rx_skb(sc, skb);

    if (sta == NULL)
    {
        if (skb->len > AMPDU_BA_FRAME_LEN)
        {
            char strbuf[256];
            struct ampdu_ba_notify_data *ba_notification =
                    (struct ampdu_ba_notify_data *) (skb->data + skb->len
                                                     - sizeof(struct ampdu_ba_notify_data));
            _dump_BA_notification(strbuf, ba_notification);
            prn_aggr_err("BA from not connected STA (%02X-%02X-%02X-%02X-%02X-%02X) (%s)\n",
                    BA_frame->ta_addr[0], BA_frame->ta_addr[1], BA_frame->ta_addr[2],
                    BA_frame->ta_addr[3], BA_frame->ta_addr[4], BA_frame->ta_addr[5], strbuf);
        }
        dev_kfree_skb_any(skb);
        return;
    }

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    ssn = BA_frame->BA_ssn;
    sn_bit_map[0] = BA_frame->BA_sn_bit_map[0];
    sn_bit_map[1] = BA_frame->BA_sn_bit_map[1];

    tid_no = BA_frame->tid_info;

    ssv_sta_priv->ampdu_mib_total_BA_counter++;

    if (ssv_sta_priv->ampdu_tid[tid_no].state == AMPDU_STATE_STOP)
    {
        prn_aggr_err("ssv6200_ampdu_BA_handler state == AMPDU_STATE_STOP.\n");
        dev_kfree_skb_any(skb);
        return;
    }

    ssv_sta_priv->ampdu_tid[tid_no].mib.ampdu_mib_BA_counter++;

    if (skb->len <= AMPDU_BA_FRAME_LEN)
    {
        /* ToDo: Check BA without BA notification.
         * In some cases, BA comes after timeout in FW. AMPDU is retried by FW
         * but BA is passed up without BA notification after timerout handler.
         * Process such kind of BA would help BA window shift sooner.
         */
        prn_aggr_err("b %d\n", ssn);
        dev_kfree_skb_any(skb);
        return;
    }

    // Get BA notify data at end of the packet.
    ba_notification =
            (struct ampdu_ba_notify_data *) (skb->data + skb->len
                                             - sizeof(struct ampdu_ba_notify_data));
#if 0
    //if (ba_notification->seq_no[0] != ssn)
    if (1)
    {
        char strbuf[256];
        _dump_BA_notification(strbuf, ba_notification);
        prn_aggr_err("B %d %08X %08X: %s\n", ssn, sn_bit_map[0], sn_bit_map[1], strbuf);
    }
#endif

    // Process BA map
    //prn_aggr_err("B %d\n", ssn);
    aggr_num = _ba_map_walker(&(ssv_sta_priv->ampdu_tid[tid_no]), ssn,
                              sn_bit_map, ba_notification, &acked_num);
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    if (ssv_sta_priv->ampdu_tid[tid_no].debugfs_dir)
    {
        struct sk_buff *dup_skb;
        if (skb_queue_len(&ssv_sta_priv->ampdu_tid[tid_no].ba_q) > 24)
        {
            struct sk_buff *ba_skb = skb_dequeue(&ssv_sta_priv->ampdu_tid[tid_no].ba_q);
            if (ba_skb)
                dev_kfree_skb_any(ba_skb);
        }
        
        dup_skb = skb_clone(skb, GFP_ATOMIC);
        if (dup_skb)
            skb_queue_tail(&ssv_sta_priv->ampdu_tid[tid_no].ba_q, dup_skb);
    }
    #endif // CONFIG_SSV6XXX_DEBUGFS
    // Trim BA notification.
    skb_trim(skb, skb->len - sizeof(struct ampdu_ba_notify_data));
    //prn_aggr_err("B %d - %d - %d - %d\n", ssn, orig_len, skb->len, (u32)(SSV6XXX_RX_DESC_LEN));

#if 0
    total_debug_count++;
    if (ba_notification_hits != ba_notification_aggr_num)
    printk("rate[%d] firmware retry [%d] agg nums[%d] hits[%d] continue_hits[%d] \n",ba_notification.data_rate,ba_notification.retry_count,ba_notification_aggr_num,ba_notification_hits,ba_notification_continue_hits);
    else
    {
        if (ba_notification.retry_count==0)
        total_perfect_debug_count++;
        else
        total_perfect_debug_count_but_firmware_retry++;
    }
    if ((total_debug_count % 2000) == 0)
    {
        printk("Percentage %d/2000\n",total_perfect_debug_count);
        printk("firmware retry [%d] no BA[%d]\n",total_perfect_debug_count_but_firmware_retry,no_ba_debug_count);
        total_debug_count = 0;
        total_perfect_debug_count_but_firmware_retry=0;
        total_perfect_debug_count = 0;
        no_ba_debug_count = 0;
    }
#endif // 0
    // Reuse skb as event for rate control
    host_evt = (HDR_HostEvent *) skb->data;
    //host_evt->c_type = HOST_EVENT;
    host_evt->h_event = SOC_EVT_RC_AMPDU_REPORT;
    //host_evt->len = evt_size;
    report_data =
            (struct firmware_rate_control_report_data *) &host_evt->dat[0];

    memcpy(report_data, ba_notification,
           sizeof(struct firmware_rate_control_report_data));

    report_data->ampdu_len = aggr_num;
    report_data->ampdu_ack_len = acked_num;

#ifdef RATE_CONTROL_HT_PERCENTAGE_TRACE
    if((acked_num) && (acked_num != aggr_num))
    {
        int i;
        for (i = 0; i < SSV62XX_TX_MAX_RATES ; i++) {
            if(report_data->rates[i].data_rate == -1)
                break;

            if(report_data->rates[i].count == 0) {
                    printk("*********************************\n");
                    printk("       Illegal HT report         \n");
                    printk("*********************************\n");
            }
            printk("        i=[%d] rate[%d] count[%d]\n",i,report_data->rates[i].data_rate,report_data->rates[i].count);
        }
        printk("AMPDU percentage = %d%% \n",acked_num*100/aggr_num);
    }
    else if(acked_num == 0)
    {
        printk("AMPDU percentage = 0%% aggr_num=%d acked_num=%d\n",aggr_num,acked_num);
    }
#endif

    skb_queue_tail(&sc->rc_report_queue, skb);
    if (sc->rc_sample_sechedule == 0)
        queue_work(sc->rc_sample_workqueue, &sc->rc_sample_work);
} // end of - ssv6200_ampdu_BA_handler -


static void _postprocess_BA (struct ssv_softc *sc, struct ssv_sta_info *sta_info, void *param)
{
    int j;
    struct ssv_sta_priv_data *ssv_sta_priv;

    if (   (sta_info->sta == NULL)
        || ((sta_info->s_flags & STA_FLAG_VALID) == 0))
        return;

    ssv_sta_priv = (struct ssv_sta_priv_data *) sta_info->sta->drv_priv;
    for (j = 0; j < WMM_TID_NUM; j++)
    {
        AMPDU_TID *ampdu_tid = &ssv_sta_priv->ampdu_tid[j];

        if (ampdu_tid->state != AMPDU_STATE_OPERATION)
            continue;

        _collect_retry_frames(ampdu_tid);
        //prn_aggr_err("RT\n");
        ssv6200_ampdu_send_retry(sc->hw, ampdu_tid, &ampdu_tid->retry_queue,
                                 true);
        // Fill up BA window with early aggregated AMPDU
        _flush_early_ampdu_q(sc, ampdu_tid);
        //prn_aggr_err("RL\n");
        _flush_release_queue(sc->hw, &ampdu_tid->release_queue);
    }
} // end of - _postprocess_BA -


void ssv6xxx_ampdu_postprocess_BA (struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;

    ssv6xxx_foreach_sta(sc, _postprocess_BA, NULL);
}


static void ssv6200_hw_set_rx_ba_session (struct ssv_hw *sh, bool on, u8 *ta,
                                          u16 tid, u16 ssn, u8 buf_size)
{
    if (on)
    {
        u32 u32ta;
        u32ta = 0;
        u32ta |= (ta[0] & 0xff) << (8 * 0);
        u32ta |= (ta[1] & 0xff) << (8 * 1);
        u32ta |= (ta[2] & 0xff) << (8 * 2);
        u32ta |= (ta[3] & 0xff) << (8 * 3);
        SMAC_REG_WRITE(sh, ADR_BA_TA_0, u32ta);

        u32ta = 0;
        u32ta |= (ta[4] & 0xff) << (8 * 0);
        u32ta |= (ta[5] & 0xff) << (8 * 1);
        SMAC_REG_WRITE(sh, ADR_BA_TA_1, u32ta);

        SMAC_REG_WRITE(sh, ADR_BA_TID, tid);
        SMAC_REG_WRITE(sh, ADR_BA_ST_SEQ, ssn);

        SMAC_REG_WRITE(sh, ADR_BA_SB0, 0);
        SMAC_REG_WRITE(sh, ADR_BA_SB1, 0);

        //turn on ba session
        SMAC_REG_WRITE(sh, ADR_BA_CTRL, 0xb);
    }
    else
    {
        //turn off ba session
        SMAC_REG_WRITE(sh, ADR_BA_CTRL, 0x0);
    }
}

void ssv6xxx_set_ampdu_rx_add_work (struct work_struct *work)
{
    struct ssv_softc
    *sc = container_of(work, struct ssv_softc, set_ampdu_rx_add_work);

    //ssvcabrio_hw_setrxbasession(sc->sc_sh, true, sc->ba_ra_addr, sc->ba_tid, sc->ba_ssn, 64);
    ssv6200_hw_set_rx_ba_session(sc->sh, true, sc->ba_ra_addr, sc->ba_tid,
                                 sc->ba_ssn, 64);
}

void ssv6xxx_set_ampdu_rx_del_work (struct work_struct *work)
{
    struct ssv_softc
    *sc = container_of(work, struct ssv_softc, set_ampdu_rx_del_work);
    u8 addr[6] = { 0 };

    //Clear register    
    ssv6200_hw_set_rx_ba_session(sc->sh, false, addr, 0, 0, 0);
}


static void _reset_ampdu_mib (struct ssv_softc *sc, struct ssv_sta_info *sta_info, void *param)
{
    struct ieee80211_sta *sta = sta_info->sta;
    struct ssv_sta_priv_data *ssv_sta_priv;
    int i;

    ssv_sta_priv = (struct ssv_sta_priv_data *)sta->drv_priv;

    for (i = 0; i < WMM_TID_NUM; i++)
    {
        ssv_sta_priv->ampdu_tid[i].ampdu_mib_reset = 1;
    }
} // end of - _reset_ampdu_mib -


void ssv6xxx_ampdu_mib_reset (struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;

    if (sc == NULL)
        return;

    ssv6xxx_foreach_sta(sc, _reset_ampdu_mib, NULL);
}


ssize_t ampdu_tx_mib_dump (struct ssv_sta_priv_data *ssv_sta_priv,
                           char *mib_str, ssize_t length)
{
    ssize_t buf_size = length;
    ssize_t prt_size;
    int j;
    //struct ssv_softc *sc;
    struct ssv_sta_info *ssv_sta = ssv_sta_priv->sta_info;
    //struct ssv_sta_info *ssv_sta = ssv_sta_priv->sta_info, *first_ssv_sta;

    if (ssv_sta->sta == NULL)
    {
        prt_size = snprintf(mib_str, buf_size, "\n    NULL STA.\n");
        mib_str += prt_size;
        buf_size -= prt_size;
        goto mib_dump_exit;
    }

    //first_ssv_sta = ssv_sta - ssv_sta_priv->sta_idx;

    //sc = container_of(first_ssv_sta, struct ssv_softc, sta_info[0]);

    for (j = 0; j < WMM_TID_NUM; j++)
    {
        int k;
        struct AMPDU_TID_st *ampdu_tid = &ssv_sta_priv->ampdu_tid[j];
        struct AMPDU_MIB_st *ampdu_mib = &ampdu_tid->mib;

        prt_size = snprintf(mib_str, buf_size, "\n    WMM_TID %d@%d\n", j,
                            ampdu_tid->state);
        mib_str += prt_size;
        buf_size -= prt_size;
        if (ampdu_tid->state != AMPDU_STATE_OPERATION)
            continue;
        prt_size = snprintf(mib_str, buf_size, "        BA window size: %d\n",
                            ampdu_tid->ssv_baw_size);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size, "        BA window head: %d\n",
                            ampdu_tid->ssv_baw_head);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        Sending aggregated #: %d\n",
                            ampdu_tid->aggr_pkt_num);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(
                mib_str, buf_size, "        Waiting #: %d\n",
                skb_queue_len(&ampdu_tid->ampdu_skb_tx_queue));
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size, "        Early aggregated %d\n",
                            ampdu_tid->early_aggr_skb_num);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        MPDU: %d\n",
                            ampdu_mib->ampdu_mib_mpdu_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        Passed: %d\n", ampdu_mib->ampdu_mib_pass_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        Retry: %d\n",
                            ampdu_mib->ampdu_mib_retry_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        AMPDU: %d\n",
                            ampdu_mib->ampdu_mib_ampdu_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        Retry AMPDU: %d\n",
                            ampdu_mib->ampdu_mib_aggr_retry_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        BAR count: %d\n",
                            ampdu_mib->ampdu_mib_bar_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        Discard count: %d\n",
                            ampdu_mib->ampdu_mib_discard_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size,
                            "        BA count: %d\n",
                            ampdu_mib->ampdu_mib_BA_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size, "        Total BA count: %d\n",
                            ssv_sta_priv->ampdu_mib_total_BA_counter);
        mib_str += prt_size;
        buf_size -= prt_size;
        prt_size = snprintf(mib_str, buf_size, "        Aggr # count:\n");
        mib_str += prt_size;
        buf_size -= prt_size;
        for (k = 0; k <= SSV_AMPDU_aggr_num_max; k++)
        {
            prt_size = snprintf(mib_str, buf_size, "            %d: %d\n", k,
                                ampdu_mib->ampdu_mib_dist[k]);
            mib_str += prt_size;
            buf_size -= prt_size;
        }
    }
mib_dump_exit:
    return (length - buf_size);
} // end of - ampdu_tx_mib_dump -


static void _dump_ampdu_mib (struct ssv_softc *sc, struct ssv_sta_info *sta_info, void *param)
{
    struct mib_dump_data *dump_data = (struct mib_dump_data *)param;
    struct ieee80211_sta *sta;
    struct ssv_sta_priv_data *ssv_sta_priv;
    ssize_t buf_size;
    ssize_t prt_size;
    char *mib_str = dump_data->prt_buff;

    if (param == NULL)
        return;

    buf_size = dump_data->buff_size - 1;

    sta = sta_info->sta;
    if ((sta == NULL) || ((sta_info->s_flags & STA_FLAG_VALID) == 0))
        return;

    prt_size = snprintf(mib_str, buf_size,
                        "STA: %02X-%02X-%02X-%02X-%02X-%02X:\n",
                        sta->addr[0], sta->addr[1], sta->addr[2],
                        sta->addr[3], sta->addr[4], sta->addr[5]);
    mib_str += prt_size;
    buf_size -= prt_size;
    ssv_sta_priv = (struct ssv_sta_priv_data *) sta->drv_priv;
    prt_size = ampdu_tx_mib_dump(ssv_sta_priv, mib_str, buf_size);
    mib_str += prt_size;
    buf_size -= prt_size;

    dump_data->prt_len = (dump_data->buff_size - 1 - buf_size);
    dump_data->prt_buff = mib_str;
    dump_data->buff_size = buf_size;
} // end of - _dimp_ampdu_mib -


ssize_t ssv6xxx_ampdu_mib_dump (struct ieee80211_hw *hw, char *mib_str,
                                ssize_t length)
{
    struct ssv_softc *sc = hw->priv;
    ssize_t buf_size = length - 1;
    struct mib_dump_data dump_data = {mib_str, buf_size, 0};

    if (sc == NULL)
        return 0;

    ssv6xxx_foreach_sta(sc, _dump_ampdu_mib, &dump_data);

    return dump_data.prt_len;
} // end of - ssv6xxx_ampdu_mib_dump -


struct sk_buff *_alloc_ampdu_skb (struct ssv_softc *sc, struct AMPDU_TID_st *ampdu_tid, u32 len)
{
    unsigned char *payload_addr;
    u32 headroom = sc->hw->extra_tx_headroom;
    u32 offset;
    u32 cur_max_ampdu_size = SSV_GET_MAX_AMPDU_SIZE(sc->sh);
    // HCI allocate extra 48 bytes for security and 80 bytes for TX descriptor anyway.
    u32 extra_room = sc->sh->tx_desc_len * 2 + 48;
    u32 max_physical_len =   (len && ((len + extra_room) < cur_max_ampdu_size))
                           ? (len + extra_room)
                           : cur_max_ampdu_size;
    u32 skb_len = max_physical_len + headroom + 3; // Add 3 such that data can later be aligned to 4-byte boundary.
    struct sk_buff *ampdu_skb = dev_alloc_skb(skb_len);
    struct ampdu_hdr_st *ampdu_hdr;

    if (ampdu_skb == NULL)
    {
        dev_err(sc->dev, "AMPDU allocation of size %d(%d) failed\n", len, skb_len);
        return NULL;
    }

    payload_addr = ampdu_skb->data + headroom - sc->sh->tx_desc_len;
    offset = ((size_t) payload_addr) % 4U;
    if (offset)
    {
        // Debug code.
        printk(KERN_ERR "Align AMPDU data %d\n", offset);
        skb_reserve(ampdu_skb, headroom + 4 - offset);
    }
    else
        skb_reserve(ampdu_skb, headroom);

    ampdu_hdr = (struct ampdu_hdr_st *) ampdu_skb->head;
    skb_queue_head_init(&ampdu_hdr->mpdu_q);
    ampdu_hdr->max_size = max_physical_len - extra_room;
    ampdu_hdr->size = 0;
    ampdu_hdr->ampdu_tid = ampdu_tid;
    memset(ampdu_hdr->ssn, 0xFF, sizeof(ampdu_hdr->ssn));
    ampdu_hdr->mpdu_num = 0;

    return ampdu_skb;
} // end of - _alloc_ampdu_skb -

bool _is_skb_q_empty (struct ssv_softc *sc, struct sk_buff *skb)
{
    u32 ac = skb_get_queue_mapping(skb);
    u32 hw_txqid = sc->tx.hw_txqid[ac];
    return AMPDU_HCI_Q_EMPTY(sc->sh, hw_txqid);
} // end of - is_skb_q_empty -


static u32 _check_timeout (struct AMPDU_TID_st *ampdu_tid)
{
    u32             ssn, end_ssn;
    unsigned long   check_jiffies = jiffies;
    u32             has_retry = 0;

    ssn = ampdu_tid->ssv_baw_head;

    if (ssn == SSV_ILLEGAL_SN)
        return 0;

    end_ssn = (ampdu_tid->ssv_baw_head + SSV_AMPDU_BA_WINDOW_SIZE)
              % SSV_AMPDU_MAX_SSN;

    do {
        struct sk_buff         *skb = INDEX_PKT_BY_SSN(ampdu_tid, ssn);
        struct SKB_info_st     *skb_info;
        unsigned long           cur_jiffies;
        unsigned long           timeout_jiffies;
        u32                     delta_ms;

        if (skb == NULL)
            break;

        skb_info = (SKB_info *) (skb->head);
        cur_jiffies = jiffies;
        timeout_jiffies = skb_info->aggr_timestamp + msecs_to_jiffies(BA_WAIT_TIMEOUT);
        if (   (skb_info->ampdu_tx_status != AMPDU_ST_SENT)
            || time_before(cur_jiffies, timeout_jiffies))
            break;

        delta_ms = jiffies_to_msecs(cur_jiffies - skb_info->aggr_timestamp);
        prn_aggr_err("rt S%d-T%d-%d (%u)\n",
                     ((struct ssv_sta_priv_data *)skb_info->sta->drv_priv)->sta_idx,
                     ampdu_tid->tidno, ssn,
                     delta_ms);
        if (delta_ms > 1000)
        {
            prn_aggr_err("Last checktime %lu - %lu = %u\n",
                         check_jiffies, ampdu_tid->timestamp,
                         jiffies_to_msecs(check_jiffies - ampdu_tid->timestamp));
        }
        has_retry += _mark_skb_retry(skb_info, skb);

        INC_PKT_SN(ssn);
    } while (ssn != end_ssn);

    ampdu_tid->timestamp = check_jiffies;

    return has_retry;
} // end of - _check_timeout -


void ssv6xxx_ampdu_check_timeout (struct ieee80211_hw *hw)
{
    struct ssv_softc *sc = hw->priv;
    struct AMPDU_TID_st *cur_AMPDU_TID;

    if (!list_empty(&sc->tx.ampdu_tx_que))
    {
        list_for_each_entry_rcu(cur_AMPDU_TID, &sc->tx.ampdu_tx_que, list)
        {
            u32 has_retry;

            if (cur_AMPDU_TID->state != AMPDU_STATE_OPERATION)
                continue;
            has_retry = _check_timeout(cur_AMPDU_TID);
            if (has_retry)
            {
                _collect_retry_frames(cur_AMPDU_TID);
                ssv6200_ampdu_send_retry(sc->hw, cur_AMPDU_TID, &cur_AMPDU_TID->retry_queue,
                                         true);
            }
        }
    }
    //spin_unlock(&sc->tx.ampdu_tx_que_lock);
} // end of - ssv6xxx_ampdu_check_timeout -


void ssv6xxx_ampdu_sent(struct ieee80211_hw *hw, struct sk_buff *ampdu)
{
    struct ssv_softc    *sc = hw->priv;
    struct ampdu_hdr_st *ampdu_hdr = (struct ampdu_hdr_st *) ampdu->head;
    struct sk_buff      *mpdu;
    unsigned long        cur_jiffies = jiffies;
    int                  i;
    SKB_info            *mpdu_skb_info;
    u16                  ssn;

    //unsigned long flags;

    if (ampdu_hdr->ampdu_tid->state != AMPDU_STATE_OPERATION)
        return;

    //spin_lock_irqsave(&ampdu_hdr->ampdu_tid->pkt_array_lock, flags);
    spin_lock_bh(&ampdu_hdr->ampdu_tid->pkt_array_lock);

    // Mark AMPDU skb sending timestamp.
    for (i = 0; i < ampdu_hdr->mpdu_num; i++)
    //while ((mpdu = __skb_dequeue(&ampdu_hdr->mpdu_q)) != NULL)
    {
        ssn = ampdu_hdr->ssn[i];
        mpdu = INDEX_PKT_BY_SSN(ampdu_hdr->ampdu_tid, ssn);
        if (mpdu == NULL)
        {
            dev_err(sc->dev, "T%d-%d is a NULL MPDU.\n",
                    ampdu_hdr->ampdu_tid->tidno, ssn);
            continue;
        }
        if (ampdu_skb_ssn(mpdu) != ssn)
        {
            dev_err(sc->dev, "T%d-%d does not match %d MPDU.\n",
                    ampdu_hdr->ampdu_tid->tidno, ssn, ampdu_skb_ssn(mpdu));
            continue;
        }
        mpdu_skb_info = (SKB_info *) (mpdu->head);
        mpdu_skb_info->aggr_timestamp = cur_jiffies;
        mpdu_skb_info->ampdu_tx_status = AMPDU_ST_SENT;
    }

    //spin_unlock_irqrestore(&ampdu_hdr->ampdu_tid->pkt_array_lock);
    spin_unlock_bh(&ampdu_hdr->ampdu_tid->pkt_array_lock);
} // end of - ssv6xxx_ampdu_sent -

