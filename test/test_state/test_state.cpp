#include <unity.h>
#include "state.h"

void test_state_defaults_safe(void) {
    ClusterState s;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.speed_rpm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.speed_rpm_l);
    TEST_ASSERT_FALSE(s.controller_l_seen);
    TEST_ASSERT_FALSE(s.soc_valid);
    TEST_ASSERT_FALSE(s.hv_active);
    TEST_ASSERT_FALSE(s.handshaked);
    TEST_ASSERT_EQUAL_UINT8(0, s.error1);
    TEST_ASSERT_EQUAL_UINT8(0, s.controller_status);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.speed_rpm_r);
    TEST_ASSERT_FALSE(s.controller_r_seen);
    TEST_ASSERT_FALSE(s.gear_from_can);
    TEST_ASSERT_EQUAL_UINT8(0, s.error1_r);
    TEST_ASSERT_EQUAL_UINT8(0, s.controller_status_r);
    TEST_ASSERT_FALSE(s.paddock);
    TEST_ASSERT_FALSE(s.tc_enabled);
    TEST_ASSERT_EQUAL_UINT8(0, s.regen_level);
    TEST_ASSERT_FALSE(s.debug_enabled);
    TEST_ASSERT_FALSE(s.water_pump_auto_enabled);
    TEST_ASSERT_FALSE(s.gps_fix_ok);
    TEST_ASSERT_EQUAL_UINT32(0, s.gps_last_rx_ms);
    TEST_ASSERT_FALSE(s.bms_ble_connected);
    TEST_ASSERT_EQUAL_UINT32(0, s.bms_last_rx_ms);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.bms_pack_voltage);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.bms_current);
    TEST_ASSERT_EQUAL_INT(0, s.bms_temp_c);
    TEST_ASSERT_EQUAL_UINT32(0, s.bms_remaining_mah);
    TEST_ASSERT_EQUAL_UINT8(0, s.bms_soh);
    TEST_ASSERT_EQUAL_UINT16(0, s.bms_cycles);
    TEST_ASSERT_EQUAL_UINT8(0, s.lap_count);
    TEST_ASSERT_EQUAL_UINT32(0, s.current_lap_ms);
    TEST_ASSERT_EQUAL_UINT32(0, s.last_lap_ms);
    TEST_ASSERT_EQUAL_UINT8(0, s.best_lap_count);
    TEST_ASSERT_EQUAL_UINT32(0, s.best_lap_ms);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_state_defaults_safe);
    return UNITY_END();
}
