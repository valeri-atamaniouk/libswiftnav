#include <check.h>
#include <stdio.h>
#include <math.h>

#include <libswiftnav/time.h>
#include <libswiftnav/constants.h>

#include "check_utils.h"

START_TEST(test_gpsdifftime)
{
  struct gpsdifftime_testcase {
    gps_time_t a, b;
    double dt;
  } testcases[] = {
    {.a = {567890.0, 1234}, .b = {567890.0, 1234}, .dt = 0},
    {.a = {567890.0, 1234}, .b = {0.0, 1234}, .dt = 567890},
    {.a = {567890.0, WN_UNKNOWN}, .b = {0.0, 1234}, .dt = -36910},
    {.a = {222222.0, 2222}, .b = {2222.0, WN_UNKNOWN}, .dt = 220000},
    {.a = {444444.0, WN_UNKNOWN}, .b = {2222.0, WN_UNKNOWN}, .dt = -162578},
    {.a = {604578.0, 1000}, .b = {222.222, 1001}, .dt = -444.222},
    {.a = {604578.0, 1001}, .b = {222.222, 1000}, .dt = 1209155.778},
  };
  const double tow_tol = 1e-10;
  for (size_t i = 0;
       i < sizeof(testcases) / sizeof(struct gpsdifftime_testcase); i++) {
    double dt = gpsdifftime(&testcases[i].a, &testcases[i].b);
    fail_unless(fabs(dt - testcases[i].dt) < tow_tol,
                "gpsdifftime test case %d failed, dt = %.12f", i, dt);
  }
}
END_TEST

START_TEST(test_normalize_gps_time)
{
  gps_time_t testcases[] = {
    {0, 1234},
    {3 * DAY_SECS, 1234},
    {WEEK_SECS + DAY_SECS, 1234},
    {0 - DAY_SECS, 1234}
  };
  const double tow_tol = 1e-10;
  for (size_t i = 0;
       i < sizeof(testcases) / sizeof(gps_time_t); i++) {
    double t_original = testcases[i].wn * WEEK_SECS + testcases[i].tow;
    normalize_gps_time(&testcases[i]);
    double t_normalized = testcases[i].wn * WEEK_SECS + testcases[i].tow;
    fail_unless(fabs(t_original - t_normalized) < tow_tol,
                "normalize_gps_time test case %d failed, t_original = %.12f, "
                "t_normalized = %.12f", i, t_original, t_normalized);
  }
}
END_TEST

START_TEST(test_gps_time_match_weeks)
{
  struct gps_time_match_weeks_testcase {
    gps_time_t t, ref, ret;
  } testcases[] = {
    {.t = {0.0, WN_UNKNOWN}, .ref = {0.0, 1234}, .ret = {0.0, 1234}},
    {.t = {WEEK_SECS-1, WN_UNKNOWN}, .ref = {0.0, 1234}, .ret = {WEEK_SECS-1, 1233}},
    {.t = {0.0, WN_UNKNOWN}, .ref = {WEEK_SECS-1, 1234}, .ret = {0.0, 1235}},
    {.t = {WEEK_SECS-1, WN_UNKNOWN}, .ref = {WEEK_SECS-1, 1234}, .ret = {WEEK_SECS-1, 1234}},
    {.t = {2*DAY_SECS, WN_UNKNOWN}, .ref = {5*DAY_SECS, 1234}, .ret = {2*DAY_SECS, 1234}},
    {.t = {5*DAY_SECS, WN_UNKNOWN}, .ref = {2*DAY_SECS, 1234}, .ret = {5*DAY_SECS, 1234}},
    {.t = {0.0, WN_UNKNOWN}, .ref = {WEEK_SECS/2, 1234}, .ret = {0.0, 1234}},
    {.t = {WEEK_SECS/2, WN_UNKNOWN}, .ref = {0.0, 1234}, .ret = {WEEK_SECS/2, 1234}},
    {.t = {WEEK_SECS/2+1, WN_UNKNOWN}, .ref = {0.0, 1234}, .ret = {WEEK_SECS/2+1, 1233}},
    {.t = {0.0, WN_UNKNOWN}, .ref = {WEEK_SECS/2+1, 1234}, .ret = {0.0, 1235}},
  };
  for (size_t i = 0;
       i < sizeof(testcases) / sizeof(struct gps_time_match_weeks_testcase); i++) {
    gps_time_match_weeks(&testcases[i].t, &testcases[i].ref);
    fail_unless(testcases[i].t.wn == testcases[i].ret.wn,
                "gps_time_match_weeks test case %d failed, t.wn = %d, ret.wn = %d",
                i, testcases[i].t.wn, testcases[i].ret.wn);
    fail_unless(testcases[i].t.tow == testcases[i].ret.tow,
                "gps_time_match_weeks test case %d failed, t.tow = %.12f, ret.tow = %.12f",
                i, testcases[i].t.tow, testcases[i].ret.tow);
  }
}
END_TEST

START_TEST(test_gps_adjust_week_cycle)
{
  struct gps_adjust_week_cycle_testcase {
    u16 wn_raw, ret;
  } testcases[] = {
    {.wn_raw = 0, .ret = 2048},
    {.wn_raw = 1023, .ret =2047},
    {.wn_raw = GPS_WEEK_REFERENCE % 1024, .ret = GPS_WEEK_REFERENCE},
    {.wn_raw = GPS_WEEK_REFERENCE % 1024 + 1, .ret = GPS_WEEK_REFERENCE + 1},
    {.wn_raw = GPS_WEEK_REFERENCE % 1024 - 1, .ret = GPS_WEEK_REFERENCE + 1023},
    {.wn_raw = GPS_WEEK_REFERENCE, .ret = GPS_WEEK_REFERENCE},
    {.wn_raw = GPS_WEEK_REFERENCE + 1, .ret = GPS_WEEK_REFERENCE + 1},
  };
  const u16 wn_ref = GPS_WEEK_REFERENCE;
  for (size_t i = 0;
       i < sizeof(testcases) / sizeof(struct gps_adjust_week_cycle_testcase); i++) {
    u16 wn = gps_adjust_week_cycle(testcases[i].wn_raw, wn_ref);
    fail_unless(wn == testcases[i].ret,
                "gps_adjust_week_cycle test case %d failed, wn = %d, ret = %d",
                i, wn, testcases[i].ret);
  }
}
END_TEST

Suite* time_test_suite(void)
{
  Suite *s = suite_create("Time handling");

  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_gpsdifftime);
  tcase_add_test(tc_core, test_normalize_gps_time);
  tcase_add_test(tc_core, test_gps_time_match_weeks);
  tcase_add_test(tc_core, test_gps_adjust_week_cycle);
  suite_add_tcase(s, tc_core);

  return s;
}
