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

#include <stdio.h>
#include <check.h>
#include <math.h>
#include <libswiftnav/nav_msg_glo.h>

nav_msg_glo_t n;


START_TEST(test_extract_glo_word)
{
  u32 ret = 0;
  nav_msg_init_glo(&n);
  n.string_bits[0] = 5;
  n.string_bits[1] = 5;
  n.string_bits[2] = 5;
  ret = extract_word_glo(&n,1,32);
  fail_unless(ret == 5, "1. %x, expected %x", ret, 5);
  ret = extract_word_glo(&n,33,3);
  fail_unless(ret == 5, "2. %x, expected %x", ret, 5);
  ret = extract_word_glo(&n,65,3);
  fail_unless(ret == 5, "3. %x, expected %x", ret, 5);

  n.string_bits[0] = 0x12345678;
  n.string_bits[1] = 0xdeadbeef;
  n.string_bits[2] = 0x87654321;
  ret = extract_word_glo(&n,1,32);
  fail_unless(ret == 0x12345678, "4. %x, expected %x", ret, 0x12345678);
  ret = extract_word_glo(&n,33,32);
  fail_unless(ret == 0xdeadbeef, "5. %x, expected %x", ret, 0xdeadbeef);
  ret = extract_word_glo(&n,65,32);
  fail_unless(ret == 0x87654321, "6. %x, expected %x", ret, 0x87654321);
  ret = extract_word_glo(&n,49,4);
  fail_unless(ret == 0xd, "7. %x, expected %x", ret, 0xd);
  
  n.string_bits[0] = 0xbeef0000;
  n.string_bits[1] = 0x4321dead;
  n.string_bits[2] = 0x00008765;
  ret = extract_word_glo(&n,17,32);
  fail_unless(ret == 0xdeadbeef, "8. %x, expected %x", ret, 0xdeadbeef);
  ret = extract_word_glo(&n,49,32);
  fail_unless(ret == 0x87654321, "9. %x, expected %x", ret, 0x87654321);
  ret = extract_word_glo(&n,49,16);
  fail_unless(ret == 0x4321, "10. %x, expected %x", ret, 0x4321);
}
END_TEST

START_TEST(test_extract_real_glo_word)
{
  u32 ret = 0;
  u8 sign = 0;
  double f_ret;
  nav_msg_init_glo(&n);
  //parse string 1
  n.string_bits[0] = 0xc3a850b5;
  n.string_bits[1] = 0x96999b05;
  n.string_bits[2] = 0x010743;
  ret = extract_word_glo(&n,85,1);
  fail_unless(ret == 0, "1. %x, expected %x", ret, 0);
  ret = extract_word_glo(&n,81,4);
  fail_unless(ret == 1, "2. %x, expected %x", ret, 1);
  printf("STRING %u\n",ret);
  ret = extract_word_glo(&n,9,26);
  sign = extract_word_glo(&n,9+26,1);
  f_ret = sign ? -1.0*ret*pow(2,-11)*1000.0 : ret*pow(2,-11)*1000.0;
  printf("x = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,41,23);
  sign = extract_word_glo(&n,41+23,1);
  f_ret = sign ? -1.0*ret*pow(2,-20)*1000.0 : ret*pow(2,-20)*1000.0;
  printf("Vx = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,36,4);
  sign = extract_word_glo(&n,36+4,1);
  f_ret = sign ? -1.0*ret*pow(2,-30)*1000.0 : ret*pow(2,-30)*1000.0;
  printf("Ax = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,65,12);
  printf("row Tk = %x\n",ret);
  ret = extract_word_glo(&n,65,5);
  printf("Tk(hours) = %u\n",ret);
  ret = extract_word_glo(&n,70,6);
  printf("Tk(mins) = %u\n",ret);
  ret = extract_word_glo(&n,76,1);
  printf("Tk(sec) = %u\n",ret);
  //parse string 2
  n.string_bits[0] = 0xd9c15f66;
  n.string_bits[1] = 0xa5256204;
  n.string_bits[2] = 0x021760;
  ret = extract_word_glo(&n,85,1);
  fail_unless(ret == 0, "1. %x, expected %x", ret, 0);
  ret = extract_word_glo(&n,81,4);
  fail_unless(ret == 2, "2. %x, expected %x", ret, 2);
  printf("STRING %u\n",ret);
  ret = extract_word_glo(&n,9,26);
  sign = extract_word_glo(&n,9+26,1);
  f_ret = sign ? -1.0*ret*pow(2,-11)*1000.0 : ret*pow(2,-11)*1000.0;
  printf("y = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,41,23);
  sign = extract_word_glo(&n,41+23,1);
  f_ret = sign ? -1.0*ret*pow(2,-20)*1000.0 : ret*pow(2,-20)*1000.0;
  printf("Vy = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,36,4);
  sign = extract_word_glo(&n,36+4,1);
  f_ret = sign ? -1.0*ret*pow(2,-30)*1000.0 : ret*pow(2,-30)*1000.0;
  printf("Ay = %f(%x, %u)\n",f_ret,ret,sign);
  //parse string 3
  n.string_bits[0] = 0x6d0e3123;
  n.string_bits[1] = 0x9d60899a;
  n.string_bits[2] = 0x038026;
  ret = extract_word_glo(&n,85,1);
  fail_unless(ret == 0, "1. %x, expected %x", ret, 0);
  ret = extract_word_glo(&n,81,4);
  fail_unless(ret == 3, "2. %x, expected %x", ret, 3);
  printf("STRING %u\n",ret);
  ret = extract_word_glo(&n,9,26);
  sign = extract_word_glo(&n,9+26,1);
  f_ret = sign ? -1.0*ret*pow(2,-11)*1000.0 : ret*pow(2,-11)*1000.0;
  printf("z = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,41,23);
  sign = extract_word_glo(&n,41+23,1);
  f_ret = sign ? -1.0*ret*pow(2,-20)*1000.0 : ret*pow(2,-20)*1000.0;
  printf("Vz = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,36,4);
  sign = extract_word_glo(&n,36+4,1);
  f_ret = sign ? -1.0*ret*pow(2,-30)*1000.0 : ret*pow(2,-30)*1000.0;
  printf("Az = %f(%x, %u)\n",f_ret,ret,sign);
  ret = extract_word_glo(&n,66,2);
  printf("P = %u\n",ret);
  //parse string 4
  n.string_bits[0] = 0x00344918;
  n.string_bits[1] = 0x1cc00000;
  n.string_bits[2] = 0x04865d;
  ret = extract_word_glo(&n,85,1);
  fail_unless(ret == 0, "1. %x, expected %x", ret, 0);
  ret = extract_word_glo(&n,81,4);
  fail_unless(ret == 4, "2. %x, expected %x", ret, 4);
  printf("STRING %u\n",ret);
  ret = extract_word_glo(&n,16,11);
  printf("Nt = %u\n",ret);
  //parse string 5
  n.string_bits[0] = 0x40000895;
  n.string_bits[1] = 0x3;
  n.string_bits[2] = 0x050d10;
  ret = extract_word_glo(&n,85,1);
  fail_unless(ret == 0, "1. %x, expected %x", ret, 0);
  ret = extract_word_glo(&n,81,4);
  fail_unless(ret == 5, "2. %x, expected %x", ret, 5);
  printf("STRING %u\n",ret);
  ret = extract_word_glo(&n,38,32);
  printf("Tau c = %u\n",ret);
  ret = extract_word_glo(&n,32,5);
  printf("N4 = %u\n",ret);
  ret = extract_word_glo(&n,10,22);
  printf("Tau GPS = %u\n",ret);
  ret = extract_word_glo(&n,70,11);
  printf("Na = %u\n",ret);
}
END_TEST
Suite* glo_decoder_test_suite(void)
{
  Suite *s = suite_create("GLO decoder");
  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_extract_glo_word);
  tcase_add_test(tc_core, test_extract_real_glo_word);
  suite_add_tcase(s, tc_core);

  return s;
}
