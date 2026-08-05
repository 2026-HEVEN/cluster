#include <unity.h>
#include "modules/hmi_input.h"

void test_paddock_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({true, false, false, false}).paddock);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false}).paddock);
}
void test_tc_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, true, false, false}).tc_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false}).tc_enabled);
}
void test_regen_auto_toggle(void) {
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false}).regen_auto_enabled);
    TEST_ASSERT_TRUE(hmi_compute({false, false, true, false}).regen_auto_enabled);
}
void test_debug_passthrough(void) {
    TEST_ASSERT_TRUE(hmi_compute({false, false, false, true}).debug_enabled);
    TEST_ASSERT_FALSE(hmi_compute({false, false, false, false}).debug_enabled);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paddock_passthrough);
    RUN_TEST(test_tc_passthrough);
    RUN_TEST(test_regen_auto_toggle);
    RUN_TEST(test_debug_passthrough);
    return UNITY_END();
}
