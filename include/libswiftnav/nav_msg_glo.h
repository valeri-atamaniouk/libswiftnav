/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_NAV_MSG_GLO_H
#define LIBSWIFTNAV_NAV_MSG_GLO_H

#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>

#define NAV_MSG_GLO_STRING_BITS_LEN 3 /* Buffer 96 nav bits. */

/** Time mark in GLO nav string, GLO ICD, pg. 16 */
#define GLO_TM (0x3E375096)
#define GLO_TM_LEN 30

typedef enum {
  INVALID = -1,
  SYNC_TM,
  GET_DATA_BIT
} glo_decode_machine;

typedef struct {
  u32 string_bits[NAV_MSG_GLO_STRING_BITS_LEN];
  u32 tm_buff;
  u16 current_head_bit_index;
  u8 next_string_id;
  u8 meander_bits_cnt;
  u8 manchester;
  glo_decode_machine state;
} nav_msg_glo_t;

void nav_msg_init_glo(nav_msg_glo_t *n);
s8 process_string_glo(nav_msg_glo_t *n, ephemeris_t *e);
u32 extract_word_glo(const nav_msg_glo_t *n, u16 bit_index, u8 n_bits); //TODO: remove after tests
s8 nav_msg_update_glo(nav_msg_glo_t *n, bool bit_val);

#endif /* LIBSWIFTNAV_NAV_MSG_GLO_H */
