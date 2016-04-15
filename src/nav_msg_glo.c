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
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <libswiftnav/nav_msg_glo.h>

void nav_msg_init_glo(nav_msg_glo_t *n)
{
  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(nav_msg_glo_t));
  n->next_string_id = 1;
}

/** Extract a word of n_bits length (n_bits <= 32) at position bit_index into
 * the subframe. Takes account of the offset stored in n, and the circular
 * nature of the n->subframe_bits buffer.
 * Refer to bit index to Table 4.6 in GLO ICD 5.1 (pg. 34)
 * \param n pointer to GLO nav message structure to be parsed
 * \param bit_index number of bit to the extract process start with. Range [1..85]
 * \param n_bits how many bits should be extract [1..32]
 * \return word extracted from navigation string */
u32 extract_word_glo(nav_msg_glo_t *n, u16 bit_index, u8 n_bits)
{
  assert(n_bits <= 32 && n_bits > 0);
  assert(bit_index <= 85 && bit_index > 0);

  /* Extract a word of n_bits length (n_bits <= 32) at position bit_index into
   * the GLO string.*/
  bit_index--;
  u32 word = 0;
  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  if (bix_lo + n_bits <= 32) {
    word = n->string_bits[bix_hi] >> bix_lo;
    word &= (0xffffffff << (32-n_bits)) >> (32-n_bits);
  } else {
    u8 s = 32-bix_lo;
    word = extract_word_glo(n, bit_index+1, s) |
           extract_word_glo(n, bit_index+1 + s, n_bits - s) << s;
  }

  return word;
}
#if 0
static s8 process_string_glo(nav_msg_t *n, ephemeris_t *e)
{
  /* Extract dummy bit from string */
  /* sanity check dummy bit */
  /* Extract string number */

  /* Extract time stamp from a sting */
  u32 ts = extract_word(n, 0, 30, 0);
  if (nav_parity(&sf_word2)) {
    log_info_sid(e->sid, "subframe parity mismatch (word 2)");
    n->gps.subframe_start_index = 0;  // Mark the subframe as processed
    n->gps.next_subframe_id = 1;      // Make sure we start again next time
    return -2;
  }

  return 0;
}
#endif
