#include <unity.h>
#include "modules/vess.h"

void test_invalid_throttle_outputs_neutral(void) {
    const VessOutput out = vess_compute({25.0f, false, 2});
    TEST_ASSERT_EQUAL_UINT16(1500, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(0, out.throttle_percent);
}

void test_zero_throttle_outputs_neutral(void) {
    const VessOutput out = vess_compute({0.0f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(1500, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(0, out.throttle_percent);
}

void test_forward_throttle_maps_to_forward_pwm(void) {
    const VessOutput out = vess_compute({50.0f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(1750, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(50, out.throttle_percent);
}

void test_forward_pwm_saturates_at_full_scale(void) {
    const VessOutput out = vess_compute({120.0f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(2000, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(100, out.throttle_percent);
}

void test_reverse_gear_uses_reverse_pwm_direction(void) {
    const VessOutput out = vess_compute({50.0f, true, 1});
    TEST_ASSERT_EQUAL_UINT16(1250, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(-50, out.throttle_percent);
}

void test_neutral_gear_ignores_valid_throttle(void) {
    const VessOutput out = vess_compute({75.0f, true, 0});
    TEST_ASSERT_EQUAL_UINT16(1500, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(0, out.throttle_percent);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_invalid_throttle_outputs_neutral);
    RUN_TEST(test_zero_throttle_outputs_neutral);
    RUN_TEST(test_forward_throttle_maps_to_forward_pwm);
    RUN_TEST(test_forward_pwm_saturates_at_full_scale);
    RUN_TEST(test_reverse_gear_uses_reverse_pwm_direction);
    RUN_TEST(test_neutral_gear_ignores_valid_throttle);
    return UNITY_END();
}

