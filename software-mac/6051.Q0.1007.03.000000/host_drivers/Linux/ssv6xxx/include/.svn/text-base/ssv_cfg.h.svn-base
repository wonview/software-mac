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
    u32     def_chan;

    //0-26M 1-40M 2-24M
    u32     crystal_type;
	
    /**
     * The mac address of Wifi STA .
     */
    u8      maddr[2][6];
    u32     n_maddr;
    // Force to use WPA2 only such that all virtual interfaces use hardware security.
    u32     use_wpa2_only;
    // Ignore reset event in AP mode
    u32     ignore_reset_in_ap;

    //E-fuse configuration
    u32     chip_id;
    u32     r_calbration_result;
    u32     sar_result;
    u32     crystal_frequency_offset;
    //u16 iq_calbration_result;
    u32     tx_power_index_1;
    u32     tx_power_index_2;
    u32     chip_identity;

    u32     wifi_tx_gain_level_gn;
    u32     wifi_tx_gain_level_b;
};




#endif /* _SSV_CFG_H_ */

