#include <unity.h>
#include "modules/hmi_input.h"

void test_paddock_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({true, false, false, false, false, false}).paddock);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false, false, false}).paddock);
}
void test_tc_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, true, false, false, false, false}).tc_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false, false, false}).tc_enabled);
}
void test_regen_rotary_bits(void) {
    TEST_ASSERT_EQUAL_UINT8(0, hmi_compute({false, false, false, false, false, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(1, hmi_compute({false, false, true, false, false, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(2, hmi_compute({false, false, false, true, false, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(3, hmi_compute({false, false, true, true, false, false}).regen_level);
}
void test_debug_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, false, false, false, true, false}).debug_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false, false, false}).debug_enabled);
}
void test_water_pump_auto_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, false, false, false, false, true}).water_pump_auto_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false, false, false}).water_pump_auto_enabled);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paddock_passthrough);
    RUN_TEST(test_tc_passthrough);
    RUN_TEST(test_regen_rotary_bits);
    RUN_TEST(test_debug_passthrough);
    RUN_TEST(test_water_pump_auto_passthrough);
    return UNITY_END();
}
