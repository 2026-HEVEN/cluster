#include <unity.h>
#include "modules/vess.h"

void test_invalid_speed_outputs_neutral(void) {
    const VessOutput out = vess_compute({25.0f, false, 2});
    TEST_ASSERT_EQUAL_UINT16(1500, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(0, out.throttle_percent);
}

void test_low_speed_outputs_neutral(void) {
    const VessOutput out = vess_compute({0.3f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(1500, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(0, out.throttle_percent);
}

void test_forward_speed_maps_to_forward_pwm(void) {
    const VessOutput out = vess_compute({20.0f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(1750, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(50, out.throttle_percent);
}

void test_forward_pwm_saturates_at_full_scale(void) {
    const VessOutput out = vess_compute({80.0f, true, 2});
    TEST_ASSERT_EQUAL_UINT16(2000, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(100, out.throttle_percent);
}

void test_reverse_gear_uses_reverse_pwm_direction(void) {
    const VessOutput out = vess_compute({20.0f, true, 1});
    TEST_ASSERT_EQUAL_UINT16(1250, out.pulse_us);
    TEST_ASSERT_EQUAL_INT16(-50, out.throttle_percent);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_invalid_speed_outputs_neutral);
    RUN_TEST(test_low_speed_outputs_neutral);
    RUN_TEST(test_forward_speed_maps_to_forward_pwm);
    RUN_TEST(test_forward_pwm_saturates_at_full_scale);
    RUN_TEST(test_reverse_gear_uses_reverse_pwm_direction);
    return UNITY_END();
}

