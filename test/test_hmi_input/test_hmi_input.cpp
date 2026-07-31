#include <unity.h>
#include "modules/hmi_input.h"

void test_paddock_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({true, false, 0, false}).paddock);
    TEST_ASSERT_FALSE(hmi_compute({false, false, 0, false}).paddock);
}
void test_tc_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, true, 0, false}).tc_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, 0, false}).tc_enabled);
}
void test_regen_level_from_rotary(void) {
    TEST_ASSERT_EQUAL_UINT8(0, hmi_compute({false, false, 0, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(1, hmi_compute({false, false, 1, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(2, hmi_compute({false, false, 2, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(3, hmi_compute({false, false, 3, false}).regen_level);
    TEST_ASSERT_EQUAL_UINT8(3, hmi_compute({false, false, 7, false}).regen_level);
}
void test_debug_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, false, 0, true}).debug_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, 0, false}).debug_enabled);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paddock_passthrough);
    RUN_TEST(test_tc_passthrough);
    RUN_TEST(test_regen_level_from_rotary);
    RUN_TEST(test_debug_passthrough);
    return UNITY_END();
}
