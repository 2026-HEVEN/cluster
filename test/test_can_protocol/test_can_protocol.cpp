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
    encode_cluster_command({false, true, 3, true, true}, out);
    TEST_ASSERT_EQUAL_UINT8(0, out[0]);   // reserved: gear is handled by VCU
    TEST_ASSERT_EQUAL_UINT8(0x1F, out[1]); // TC + regen level 3 + debug + water pump auto
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0x01);   // paddock off
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0xFE);    // remaining flags reserved
}

void test_encode_paddock_bit(void) {
    uint8_t out[8];
    encode_cluster_command({true, false, 0, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(1, out[2] & 0x01);   // paddock on
}
void test_encode_regen_level_bits(void) {
    uint8_t out[8];
    encode_cluster_command({false, false, 0, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x06);
    encode_cluster_command({false, false, 1, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1] & 0x06);
    encode_cluster_command({false, false, 2, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x04, out[1] & 0x06);
    encode_cluster_command({false, false, 3, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x06, out[1] & 0x06);
    encode_cluster_command({false, false, 9, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x06, out[1] & 0x06);
}
void test_encode_water_pump_auto_bit(void) {
    uint8_t out[8];
    encode_cluster_command({false, false, 0, false, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x10);
    encode_cluster_command({false, false, 0, false, true}, out);
    TEST_ASSERT_EQUAL_UINT8(0x10, out[1] & 0x10);
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

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_torque_offset);
    RUN_TEST(test_cluster_cmd_id);
    RUN_TEST(test_vcu_cluster_status_id);
    RUN_TEST(test_vcu_vehicle_speed_id);
    RUN_TEST(test_cluster_bms_ids);
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
    RUN_TEST(test_encode_water_pump_auto_bit);
    RUN_TEST(test_encode_cluster_bms_status);
    RUN_TEST(test_encode_cluster_bms_detail);
    return UNITY_END();
}
