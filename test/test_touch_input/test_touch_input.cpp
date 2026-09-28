#include <unity.h>
#include "modules/touch_input.h"

namespace {
TouchRawSample down(int16_t x, int16_t y) { return {true, x, y}; }
TouchRawSample up() { return {false, 0, 0}; }

// Feed one sample every 10 ms; return the number of taps and the last one.
int run(TouchTracker &t, const TouchRawSample *s, int n, uint32_t &now, TouchTap &last) {
    int taps = 0;
    for (int i = 0; i < n; ++i) {
        const TouchTap tap = t.feed(s[i], now);
        if (tap.valid) { ++taps; last = tap; }
        now += 10;
    }
    return taps;
}
}

void test_tap_fires_on_press_before_release(void) {
    TouchTracker t;
    uint32_t now = 0;
    TEST_ASSERT_FALSE(t.feed(down(1000, 2000), now).valid);
    const TouchTap tap = t.feed(down(1000, 2000), now + 10);
    TEST_ASSERT_TRUE(tap.valid);
    TEST_ASSERT_EQUAL_INT16(1000, tap.raw_x);
}

void test_quick_two_sample_tap_is_accepted(void) {
    TouchTracker t;
    TouchRawSample s[] = {down(1000, 2000), down(1000, 2000), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(1, run(t, s, 9, now, last));
}

void test_clean_press_gives_one_tap(void) {
    TouchTracker t;
    TouchRawSample s[] = {down(1000, 2000), down(1000, 2000), down(1000, 2000), down(1000, 2000),
                          up(), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(1, run(t, s, 12, now, last));
    TEST_ASSERT_EQUAL_INT16(1000, last.raw_x);
    TEST_ASSERT_EQUAL_INT16(2000, last.raw_y);
}

void test_single_sample_spike_is_ignored(void) {
    TouchTracker t;
    TouchRawSample s[] = {down(1000, 2000), up(), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(0, run(t, s, 9, now, last));
}

void test_pressure_dropout_while_held_is_one_tap(void) {
    // Pressure dips below threshold for one sample mid-press: the old code
    // fired a tap on each dip and navigated several pages.
    TouchTracker t;
    TouchRawSample s[] = {down(1000, 2000), down(1000, 2000), down(1000, 2000), up(),
                          down(1000, 2000), down(1000, 2000), up(), down(1000, 2000),
                          up(), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(1, run(t, s, 16, now, last));
}

void test_first_contact_sample_and_outliers_do_not_move_tap(void) {
    TouchTracker t;
    TouchRawSample s[] = {down(3000, 3000), down(1000, 2000), down(1010, 1990), down(2500, 500),
                          down(990, 2010), down(1000, 2000), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(1, run(t, s, 13, now, last));
    TEST_ASSERT_INT16_WITHIN(15, 1000, last.raw_x);
    TEST_ASSERT_INT16_WITHIN(15, 2000, last.raw_y);
}

void test_press_right_after_tap_is_locked_out(void) {
    TouchTracker t;
    TouchRawSample s[] = {down(1000, 2000), down(1000, 2000), down(1000, 2000),
                          up(), up(), up(), up(), up(), up(), up(),
                          down(1000, 2000), down(1000, 2000), down(1000, 2000),
                          up(), up(), up(), up(), up(), up(), up(), up()};
    uint32_t now = 0; TouchTap last;
    TEST_ASSERT_EQUAL_INT(1, run(t, s, 21, now, last));
}

void test_long_hold_is_one_tap(void) {
    TouchTracker t;
    uint32_t now = 0;
    int taps = 0;
    for (int i = 0; i < 300; ++i) { if (t.feed(down(1000, 2000), now).valid) ++taps; now += 10; }
    for (int i = 0; i < 10; ++i) { if (t.feed(up(), now).valid) ++taps; now += 10; }
    TEST_ASSERT_EQUAL_INT(1, taps);
}

void test_default_calibration_matches_previous_mapping(void) {
    TouchCalibration cal;
    int x, y;
    touch_map(cal, 3900, 200, x, y);
    TEST_ASSERT_INT_WITHIN(2, 0, x);
    TEST_ASSERT_INT_WITHIN(2, 0, y);
    touch_map(cal, 2050, 1850, x, y);
    TEST_ASSERT_INT_WITHIN(2, 160, x);
    TEST_ASSERT_INT_WITHIN(2, 107, y);
}

void test_map_clamps_to_screen(void) {
    TouchCalibration cal;
    int x, y;
    touch_map(cal, 0, 4095, x, y);
    TEST_ASSERT_EQUAL_INT(319, x);
    TEST_ASSERT_EQUAL_INT(239, y);
}

// Simulated panel: raw_x grows downward, raw_y shrinks rightward (swapped + inverted).
void panel(int sx, int sy, int16_t &rx, int16_t &ry) {
    rx = (int16_t)(300 + sy * 14);
    ry = (int16_t)(3800 - sx * 11);
}

void test_calibrator_detects_swapped_axes(void) {
    TouchCalibrator c;
    c.start();
    TouchCalibrator::Result r = TouchCalibrator::Result::InProgress;
    for (int i = 0; i < TouchCalibrator::POINTS; ++i) {
        int tx, ty; int16_t rx, ry;
        c.target(i, tx, ty);
        panel(tx, ty, rx, ry);
        r = c.tap(rx, ry);
    }
    TEST_ASSERT_TRUE(r == TouchCalibrator::Result::Done);
    TEST_ASSERT_TRUE(c.result().swap_xy);
    int16_t rx, ry; int x, y;
    panel(250, 60, rx, ry);
    touch_map(c.result(), rx, ry, x, y);
    TEST_ASSERT_INT_WITHIN(2, 250, x);
    TEST_ASSERT_INT_WITHIN(2, 60, y);
}

void test_calibrator_fails_on_bad_centre_check(void) {
    TouchCalibrator c;
    c.start();
    TouchCalibrator::Result r = TouchCalibrator::Result::InProgress;
    for (int i = 0; i < TouchCalibrator::POINTS; ++i) {
        int tx, ty; int16_t rx, ry;
        c.target(i, tx, ty);
        if (i == TouchCalibrator::POINTS - 1) tx += 60;   // user missed the centre
        panel(tx, ty, rx, ry);
        r = c.tap(rx, ry);
    }
    TEST_ASSERT_TRUE(r == TouchCalibrator::Result::Failed);
    TEST_ASSERT_FALSE(c.active());
}

void test_calibrator_rejects_collapsed_axis(void) {
    TouchCalibrator c;
    c.start();
    TouchCalibrator::Result r = TouchCalibrator::Result::InProgress;
    for (int i = 0; i < 4; ++i) r = c.tap(2000, 2000);   // MISO stuck / no panel
    TEST_ASSERT_TRUE(r == TouchCalibrator::Result::Failed);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_tap_fires_on_press_before_release);
    RUN_TEST(test_quick_two_sample_tap_is_accepted);
    RUN_TEST(test_clean_press_gives_one_tap);
    RUN_TEST(test_single_sample_spike_is_ignored);
    RUN_TEST(test_pressure_dropout_while_held_is_one_tap);
    RUN_TEST(test_first_contact_sample_and_outliers_do_not_move_tap);
    RUN_TEST(test_press_right_after_tap_is_locked_out);
    RUN_TEST(test_long_hold_is_one_tap);
    RUN_TEST(test_default_calibration_matches_previous_mapping);
    RUN_TEST(test_map_clamps_to_screen);
    RUN_TEST(test_calibrator_detects_swapped_axes);
    RUN_TEST(test_calibrator_fails_on_bad_centre_check);
    RUN_TEST(test_calibrator_rejects_collapsed_axis);
    return UNITY_END();
}
