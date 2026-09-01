#include <unity.h>
#include "can_protocol.h"
#include "cluster_command.h"

// shared torque scaling (from VCU base)
void test_torque_offset(void) { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
// Cluster additions
void test_cluster_cmd_id(void) { TEST_ASSERT_EQUAL_HEX32(0x1801D0C0, CAN_ID_CLUSTER_CMD); }
void test_vcu_cluster_status_id(void) { TEST_ASSERT_EQUAL_HEX32(0x1801C0D0, CAN_ID_VCU_CLUSTER_STATUS); }
void test_vcu_vehicle_speed_id(void) { TEST_ASSERT_EQUAL_HEX32(0x1803C0D0, CAN_ID_VCU_VEHICLE_SPEED); }
void test_cluster_bms_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x18F3FFC0, CAN_ID_CLUSTER_BMS_STATUS);
    TEST_ASSERT_EQUAL_HEX32(0x18F4FFC0, CAN_ID_CLUSTER_BMS_DETAIL);
}
void test_cluster_gnss_lap_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x18F5FFC0, CAN_ID_CLUSTER_GNSS_POSITION);
    TEST_ASSERT_EQUAL_HEX32(0x18F6FFC0, CAN_ID_CLUSTER_GNSS_RTK_STATUS);
    TEST_ASSERT_EQUAL_HEX32(0x18F7FFC0, CAN_ID_CLUSTER_LAP_TIME);
    TEST_ASSERT_EQUAL_HEX32(0x18F8FFC0, CAN_ID_CLUSTER_LAP_STATUS);
}
void test_feedback_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x1801D0EF, CAN_ID_FB1_L);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0EF, CAN_ID_FB2_L);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0F0, CAN_ID_FB1_R);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0F0, CAN_ID_FB2_R);
}
// guards against a copy-paste mistake reusing the same ID for L and R
void test_feedback_ids_lr_distinct(void) {
    TEST_ASSERT_NOT_EQUAL(CAN_ID_FB1_L, CAN_ID_FB1_R);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_FB2_L, CAN_ID_FB2_R);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_FB1_L, CAN_ID_FB2_L);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_FB1_R, CAN_ID_FB2_R);
}
void test_decode_voltage(void) { TEST_ASSERT_FLOAT_WITHIN(0.01f, 48.0f, raw_to_voltage(480)); }   // 0.1V/bit
void test_decode_current(void) { TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, raw_to_current(32000)); }    // 0.1A/bit, -3200
void test_decode_temp(void)    { TEST_ASSERT_EQUAL_INT(25, raw_to_temp(65)); }                       // 1C/bit, -40
void test_decode_speed(void)   { TEST_ASSERT_EQUAL_INT(0, raw_to_speed(32000)); }                    // 1rpm/bit, -32000


void test_decode_vcu_vehicle_speed_valid(void) {
    uint8_t d[8] = {0xE8, 0x03, 1, 0, 0, 0, 0, 0};
    float kph = -1.0f;
    bool valid = false;
    decode_vcu_vehicle_speed(d, kph, valid);
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, kph);
}
void test_decode_vcu_vehicle_speed_invalid_clears_value(void) {
    uint8_t d[8] = {0xE8, 0x03, 0, 0, 0, 0, 0, 0};
    float kph = -1.0f;
    bool valid = true;
    decode_vcu_vehicle_speed(d, kph, valid);
    TEST_ASSERT_FALSE(valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kph);
}
void test_decode_vcu_vehicle_speed_zero_valid(void) {
    uint8_t d[8] = {0x00, 0x00, 1, 0, 0, 0, 0, 0};
    float kph = -1.0f;
    bool valid = false;
    decode_vcu_vehicle_speed(d, kph, valid);
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kph);
}
void test_decode_vcu_vehicle_speed_max_value(void) {
    uint8_t d[8] = {0xFF, 0xFF, 1, 0, 0, 0, 0, 0};
    float kph = 0.0f;
    bool valid = false;
    decode_vcu_vehicle_speed(d, kph, valid);
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6553.5f, kph);
}

void test_encode_config_flags(void) {
    uint8_t out[8];
    encode_cluster_command({false, true, 3, true}, out);
    TEST_ASSERT_EQUAL_UINT8(0, out[0]);   // reserved: gear is handled by VCU
    TEST_ASSERT_EQUAL_UINT8(0x0F, out[1]); // TC + regen level 3 + debug
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0x01);   // paddock off
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0xFE);    // remaining flags reserved
}

void test_encode_paddock_bit(void) {
    uint8_t out[8];
    encode_cluster_command({true, false, 0, false}, out);
    TEST_ASSERT_EQUAL_UINT8(1, out[2] & 0x01);   // paddock on
}
void test_encode_regen_level_bits(void) {
    uint8_t out[8];
    encode_cluster_command({false, false, 0, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x06);
    encode_cluster_command({false, false, 1, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1] & 0x06);
    encode_cluster_command({false, false, 2, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x04, out[1] & 0x06);
    encode_cluster_command({false, false, 3, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x06, out[1] & 0x06);
    encode_cluster_command({false, false, 9, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x06, out[1] & 0x06);
}
void test_encode_cluster_bms_status(void) {
    uint8_t out[8];
    ClusterBmsStatus bms;
    bms.valid = true;
    bms.ble_connected = true;
    bms.soc_pct = 78;
    bms.pack_voltage_v = 51.2f;
    bms.current_a = -12.3f;
    bms.temp_c = 35;
    encode_cluster_bms_status(bms, 0x42, out);

    TEST_ASSERT_EQUAL_UINT8(0x03, out[0]);      // valid + BLE connected
    TEST_ASSERT_EQUAL_UINT8(78, out[1]);        // SOC %
    TEST_ASSERT_EQUAL_UINT8(0x00, out[2]);      // 51.2 V -> 512
    TEST_ASSERT_EQUAL_UINT8(0x02, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x85, out[4]);      // -12.3 A -> 31877
    TEST_ASSERT_EQUAL_UINT8(0x7C, out[5]);
    TEST_ASSERT_EQUAL_UINT8(75, out[6]);        // 35 C + 40
    TEST_ASSERT_EQUAL_UINT8(0x42, out[7]);      // life
}
void test_encode_cluster_bms_detail(void) {
    uint8_t out[8];
    ClusterBmsStatus bms;
    bms.valid = true;
    bms.ble_connected = true;
    bms.remaining_mah = 12345;
    bms.soh_pct = 97;
    bms.cycles = 321;
    encode_cluster_bms_detail(bms, 0x43, out);

    TEST_ASSERT_EQUAL_UINT8(0x03, out[0]);
    TEST_ASSERT_EQUAL_UINT8(97, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x39, out[2]);      // remaining mAh 12345
    TEST_ASSERT_EQUAL_UINT8(0x30, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x41, out[4]);      // cycles 321
    TEST_ASSERT_EQUAL_UINT8(0x01, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x43, out[7]);
}
void test_encode_cluster_gnss_position(void) {
    uint8_t out[8];
    ClusterGnssPosition pos;
    pos.latitude_deg = 37.1234567;
    pos.longitude_deg = 127.7654321;
    encode_cluster_gnss_position(pos, out);

    TEST_ASSERT_EQUAL_UINT8(0x07, out[0]); // 371234567
    TEST_ASSERT_EQUAL_UINT8(0x97, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x20, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x16, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x31, out[4]); // 1277654321
    TEST_ASSERT_EQUAL_UINT8(0x75, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x27, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x4C, out[7]);
}
void test_encode_cluster_gnss_rtk_status(void) {
    uint8_t out[8];
    ClusterGnssRtkStatus status;
    status.gps_data_fresh = true;
    status.gps_fix_valid = true;
    status.ntrip_connected = true;
    status.rtcm_fresh = false;
    status.fix_quality = 5;
    status.rtk_state = 1;
    status.satellites = 12;
    status.hdop = 0.9f;
    status.rtcm_age_dsec = 37;
    encode_cluster_gnss_rtk_status(status, out);

    TEST_ASSERT_EQUAL_UINT8(0x07, out[0]);
    TEST_ASSERT_EQUAL_UINT8(5, out[1]);
    TEST_ASSERT_EQUAL_UINT8(1, out[2]);
    TEST_ASSERT_EQUAL_UINT8(12, out[3]);
    TEST_ASSERT_EQUAL_UINT8(9, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0, out[5]);
    TEST_ASSERT_EQUAL_UINT8(37, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0, out[7]);
}
void test_encode_cluster_lap_time(void) {
    uint8_t out[8];
    encode_cluster_lap_time(123456, 654321, out);

    TEST_ASSERT_EQUAL_UINT8(0x40, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xE2, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0xF1, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFB, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x09, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[7]);
}
void test_encode_cluster_lap_status(void) {
    uint8_t out[8];
    ClusterLapStatus lap;
    lap.best_lap_ms = 98765;
    lap.lap_count = 3;
    lap.best_lap_count = 2;
    lap.timer_running = true;
    lap.life = 0xA5;
    encode_cluster_lap_status(lap, out);

    TEST_ASSERT_EQUAL_UINT8(0xCD, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x81, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[3]);
    TEST_ASSERT_EQUAL_UINT8(3, out[4]);
    TEST_ASSERT_EQUAL_UINT8(2, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0xA5, out[7]);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_torque_offset);
    RUN_TEST(test_cluster_cmd_id);
    RUN_TEST(test_vcu_cluster_status_id);
    RUN_TEST(test_vcu_vehicle_speed_id);
    RUN_TEST(test_cluster_bms_ids);
    RUN_TEST(test_cluster_gnss_lap_ids);
    RUN_TEST(test_feedback_ids);
    RUN_TEST(test_feedback_ids_lr_distinct);
    RUN_TEST(test_decode_voltage);
    RUN_TEST(test_decode_current);
    RUN_TEST(test_decode_temp);
    RUN_TEST(test_decode_speed);
    RUN_TEST(test_decode_vcu_vehicle_speed_valid);
    RUN_TEST(test_decode_vcu_vehicle_speed_invalid_clears_value);
    RUN_TEST(test_decode_vcu_vehicle_speed_zero_valid);
    RUN_TEST(test_decode_vcu_vehicle_speed_max_value);
    RUN_TEST(test_encode_config_flags);
    RUN_TEST(test_encode_paddock_bit);
    RUN_TEST(test_encode_regen_level_bits);
    RUN_TEST(test_encode_cluster_bms_status);
    RUN_TEST(test_encode_cluster_bms_detail);
    RUN_TEST(test_encode_cluster_gnss_position);
    RUN_TEST(test_encode_cluster_gnss_rtk_status);
    RUN_TEST(test_encode_cluster_lap_time);
    RUN_TEST(test_encode_cluster_lap_status);
    return UNITY_END();
}
