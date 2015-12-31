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

#ifndef _SSV_EFUSE_H_
#define _SSV_EFUSE_H_

#include "dev.h"

struct efuse_map {
    u8 offset;
    u8 byte_cnts;
    u16 value;
};

enum efuse_data_item {
   EFUSE_R_CALIBRAION_RESULT = 1,
   EFUSE_SAR_RESULT,
   EFUSE_MAC,
   EFUSE_CRYSTAL_FREQUECY_OFFSET,
   //EFUSE_IQ_CALIBRAION_RESULT,
   EFUSE_TX_POWER_INDEX_1,
   EFUSE_TX_POWER_INDEX_2,
   EFUSE_CHIP_IDENTITY
};

void efuse_read_all_map(struct ssv_hw *sh);

#endif

