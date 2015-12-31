#ifndef _SSV_CFG_H_
#define _SSV_CFG_H_



/**
* SSV6200 Hardware Capabilities:
*
* @ SSV6200_HW_CAP_HT: hardware supports HT capability.
* @ SSV6200_HW_CAP_LDPC:
* @ SSV6200_HW_CAP_2GHZ:
* @ SSV6200_HW_CAP_5GHZ:
* @ SSV6200_HW_CAP_DFS:
* @ SSV6200_HW_CAP_SECUR:
*/
#define SSV6200_HW_CAP_HT                   0x00000001
#define SSV6200_HW_CAP_GF                   0x00000002
#define SSV6200_HW_CAP_2GHZ                 0x00000004
#define SSV6200_HW_CAP_5GHZ                 0x00000008
#define SSV6200_HW_CAP_SECURITY             0x00000010
#define SSV6200_HT_CAP_SGI_20               0x00000020
#define SSV6200_HT_CAP_SGI_40               0x00000040
#define SSV6200_HW_CAP_AP                   0x00000080
#define SSV6200_HW_CAP_P2P                  0x00000100
#define SSV6200_HW_CAP_AMPDU_RX             0x00000200
#define SSV6200_HW_CAP_AMPDU_TX             0x00000400




struct ssv6xxx_cfg {
    /**
     * ssv6200 hardware capabilities sets.
     */
    u32     hw_caps;

    /**
     * The default channel once the wifi system is up.
     */
    u8      def_chan;

    //0-26M 1-40M 2-24M
    u8      crystal_type;
	
    /**
     * The mac address of Wifi STA .
     */
    u8      maddr[2][6];
    u32     n_maddr;
    // Force to use WPA2 only such that all virtual interfaces use hardware security.
    u8      use_wpa2_only;

    //E-fuse configuration
    u8 r_calbration_result;
    u8 sar_result;
    u8 crystal_frequecy_offse;
    //u16 iq_calbration_result;
    u8 tx_power_index_1;
    u8 tx_power_index_2;
    u8 chip_identity;

    u8 wifi_tx_gain_level_gn;
    u8 wifi_tx_gain_level_b;
};




#endif /* _SSV_CFG_H_ */

