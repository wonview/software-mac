#ifndef WAPI_WPI_H
#define WAPI_WPI_H

#define WAPI_KEYID_LEN 1
#define WAPI_RESERVD_LEN 1
#define WAPI_PN_LEN 16
#define WAPI_IV_LEN (WAPI_KEYID_LEN + WAPI_RESERVD_LEN + WAPI_PN_LEN)
#define WAPI_MIC_LEN 16
#define ADDID_LEN (ETH_ALEN + ETH_ALEN)

//#define MAX_MAC_HDR_LEN         26 //QOS_MAC_HDR_LEN
//#define SUB_MSDU_HEADER_LENGTH  14
//#define SNAP_HDR_LEN            8
//#define ETHERNET_HDR_LEN          14
//#define WORD_ALIGNMENT_PAD        0
#define WAPI_IV_ICV_OFFSET (WAPI_IV_LEN + WAPI_MIC_LEN)


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

typedef enum {BFALSE = 0,
              BTRUE  = 1
} BOOL_T;

typedef enum {TV_TRUE  = 1,
              TV_FALSE = 2
} TRUTH_VALUE_T;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
//extern u8 g_wapi_oui[3];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

int lib80211_wpi_set_key(void *key, int len, u8 *seq, void *priv);
int lib80211_wpi_encrypt(struct sk_buff *mpdu, int hdr_len, void *priv);
int lib80211_wpi_decrypt(struct sk_buff *mpdu, int hdr_len, void *priv);
void *lib80211_wpi_init(int key_idx);
void lib80211_wpi_deinit(void *priv);
#ifdef MULTI_THREAD_ENCRYPT
int lib80211_wpi_encrypt_prepare(struct sk_buff *mpdu, int hdr_len, void *priv);
#endif

#endif /* WAPI_WPI_H */

