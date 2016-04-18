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

typedef struct {
  u32 string_bits[NAV_MSG_GLO_STRING_BITS_LEN];
  u8 next_string_id;
} nav_msg_glo_t;

void nav_msg_init_glo(nav_msg_glo_t *n);
s8 process_string_glo(nav_msg_glo_t *n, ephemeris_t *e);
u32 extract_word_glo(const nav_msg_glo_t *n, u16 bit_index, u8 n_bits); //TODO: remove after tests

#endif /* LIBSWIFTNAV_NAV_MSG_GLO_H */
