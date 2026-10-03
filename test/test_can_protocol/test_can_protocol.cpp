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
    TEST_ASSERT_EQUAL_HEX32(0x18F9FFC0, CAN_ID_CLUSTER_GNSS_SPEED);
    TEST_ASSERT_EQUAL_HEX32(0x18FBFFC0, CAN_ID_CLUSTER_LAP_HISTORY);
    TEST_ASSERT_EQUAL_HEX32(0x18FCFFC0, CAN_ID_CLUSTER_LAP_BATTERY);
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
void test_decode_vcu_vehicle_speed_invalid_keeps_value(void) {
    uint8_t d[8] = {0xE8, 0x03, 0, 0, 0, 0, 0, 0};
    float kph = -1.0f;
    bool valid = true;
    decode_vcu_vehicle_speed(d, kph, valid);
    TEST_ASSERT_FALSE(valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, kph);
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

void test_decode_vcu_cluster_status_paddock_feedback(void) {
    uint8_t d[8] = {2, 0x1F, 110, 125, 0, 0, 0, 0x5A};
    VcuClusterStatus status = decode_vcu_cluster_status(d);
    TEST_ASSERT_TRUE(status.gear_valid);
    TEST_ASSERT_EQUAL_UINT8(2, status.gear);
    TEST_ASSERT_TRUE(status.brake);
    TEST_ASSERT_TRUE(status.hv_active);
    TEST_ASSERT_TRUE(status.soc_valid);
    TEST_ASSERT_EQUAL_UINT8(100, status.soc_pct);
    TEST_ASSERT_TRUE(status.throttle_valid);
    TEST_ASSERT_EQUAL_UINT8(100, status.throttle_pct);
    TEST_ASSERT_TRUE(status.paddock_active);
    TEST_ASSERT_EQUAL_UINT8(0x5A, status.life);

    d[1] = 0;
    status = decode_vcu_cluster_status(d);
    TEST_ASSERT_FALSE(status.paddock_active);
    TEST_ASSERT_FALSE(status.soc_valid);
    TEST_ASSERT_FALSE(status.throttle_valid);
}

void test_encode_config_flags(void) {
    uint8_t out[8];
    encode_cluster_command({false, true, 3, true}, out);
    TEST_ASSERT_EQUAL_UINT8(0, out[0]);   // reserved: gear is handled by VCU
    TEST_ASSERT_EQUAL_UINT8(0x0B, out[1]); // TC + regen enable + debug
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0x01);   // paddock off
    TEST_ASSERT_EQUAL_UINT8(0, out[2] & 0xFE);    // remaining flags reserved
}

void test_encode_paddock_bit(void) {
    uint8_t out[8];
    encode_cluster_command({true, false, 0, false}, out);
    TEST_ASSERT_EQUAL_UINT8(1, out[2] & 0x01);   // paddock on
}
void test_encode_regen_level_as_vcu_boolean(void) {
    uint8_t out[8];
    encode_cluster_command({false, false, 0, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x06);
    encode_cluster_command({false, false, 1, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1] & 0x06);
    encode_cluster_command({false, false, 2, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1] & 0x06);
    encode_cluster_command({false, false, 3, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1] & 0x06);
    encode_cluster_command({false, false, 9, false}, out);
    TEST_ASSERT_EQUAL_UINT8(0, out[1] & 0x06);
    TEST_ASSERT_EQUAL_UINT8(0xA0, out[3]);
    for (uint8_t i=0;i<4;++i) {
        encode_cluster_command({false,false,i,false},out);
        TEST_ASSERT_EQUAL_UINT8(0xA0|i,out[3]);
    }
}
void test_ezkontrol_handshake_probe_detection(void) {
    uint8_t probe[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    TEST_ASSERT_TRUE(is_ezkontrol_handshake_probe(probe));
    probe[6] = 0x54;
    TEST_ASSERT_FALSE(is_ezkontrol_handshake_probe(probe));
}
void test_ezkontrol_handshake_ack_detection(void) {
    uint8_t ack[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    TEST_ASSERT_TRUE(is_ezkontrol_handshake_ack(ack));
    ack[0] = 0x00;
    TEST_ASSERT_FALSE(is_ezkontrol_handshake_ack(ack));
}
void test_decode_motor_target_current(void) {
    uint8_t data[8] = {0x20, 0x7D, 0, 0, 0, 0, 0, 0}; // 32032 -> 3.2 A
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.2f, decode_motor_target_current_a(data));
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
void test_encode_cluster_gnss_speed(void) {
    uint8_t out[8];
    ClusterGnssSpeed speed;
    speed.speed_kph = 123.45f;
    speed.rmc_fresh = true;
    speed.gps_fix_valid = true;
    speed.speed_valid = true;
    speed.rtk_state = 2;
    speed.fix_quality = 4;
    speed.rmc_age_dsec = 3;
    speed.life = 0x5A;
    encode_cluster_gnss_speed(speed, out);

    TEST_ASSERT_EQUAL_UINT8(0x39, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x30, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x1B, out[2]);
    TEST_ASSERT_EQUAL_UINT8(4, out[3]);
    TEST_ASSERT_EQUAL_UINT8(3, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x5A, out[7]);
}
void test_encode_cluster_gnss_speed_invalid_clears_value(void) {
    uint8_t out[8];
    ClusterGnssSpeed speed;
    speed.speed_kph = 99.0f;
    speed.rmc_fresh = true;
    speed.gps_fix_valid = false;
    speed.speed_valid = false;
    speed.rmc_age_dsec = 0xFFFF;
    encode_cluster_gnss_speed(speed, out);

    TEST_ASSERT_EQUAL_UINT8(0, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[5]);
}
void test_encode_cluster_gnss_speed_saturates(void) {
    uint8_t out[8];
    ClusterGnssSpeed speed;
    speed.speed_kph = 1000.0f;
    speed.speed_valid = true;
    encode_cluster_gnss_speed(speed, out);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[1]);
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

void test_encode_cluster_lap_history(void) {
    uint8_t out[8];
    ClusterLapHistoryFrame lap;
    lap.lap_number = 3;
    lap.valid = true;
    lap.active = true;
    lap.timer_running = true;
    lap.lap_time_ms = 0x12345678u;
    lap.session = 7;
    lap.life = 42;
    encode_cluster_lap_history(lap, out);

    TEST_ASSERT_EQUAL_UINT8(3, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x78, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x56, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x34, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x12, out[5]);
    TEST_ASSERT_EQUAL_UINT8(7, out[6]);
    TEST_ASSERT_EQUAL_UINT8(42, out[7]);
}

void test_encode_cluster_lap_battery(void) {
    uint8_t out[8];
    ClusterLapHistoryFrame lap;
    lap.lap_number = 3;
    lap.battery_valid = true;
    lap.active = true;
    lap.timer_running = true;
    lap.battery_used_x10 = 23;
    lap.session = 7;
    lap.life = 42;
    encode_cluster_lap_battery(lap, out);

    TEST_ASSERT_EQUAL_UINT8(3, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x17, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    TEST_ASSERT_EQUAL_UINT8(7, out[6]);
    TEST_ASSERT_EQUAL_UINT8(42, out[7]);
}

void test_lap_history_crossing_and_pause_resume(void) {
    ClusterLapHistory history;
    ClusterLapHistoryInput in;
    ClusterLapHistoryFrame frame;

    in.current_lap_number = 1;
    in.current_lap_ms = 200;
    in.current_battery_used_x10 = 10;
    in.current_battery_valid = true;
    in.timer_running = true;
    history.update(in);
    TEST_ASSERT_EQUAL_UINT8(1, history.session());
    TEST_ASSERT_TRUE(history.active_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(1, frame.lap_number);
    TEST_ASSERT_EQUAL_UINT32(200, frame.lap_time_ms);
    TEST_ASSERT_TRUE(frame.active && frame.timer_running);
    TEST_ASSERT_TRUE(frame.battery_valid);
    TEST_ASSERT_EQUAL_UINT16(10, frame.battery_used_x10);

    in.current_lap_number = 2;
    in.completed_lap_count = 1;
    in.current_lap_ms = 0;
    in.last_lap_ms = 58430;
    in.last_battery_used_x10 = 20;
    in.last_battery_valid = true;
    history.update(in);
    TEST_ASSERT_TRUE(history.take_transition_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(1, frame.lap_number);
    TEST_ASSERT_EQUAL_UINT32(58430, frame.lap_time_ms);
    TEST_ASSERT_TRUE(frame.valid && frame.completed);
    TEST_ASSERT_TRUE(frame.battery_valid);
    TEST_ASSERT_EQUAL_UINT16(20, frame.battery_used_x10);
    TEST_ASSERT_FALSE(frame.active);
    TEST_ASSERT_TRUE(history.active_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(2, frame.lap_number);
    TEST_ASSERT_EQUAL_UINT32(0, frame.lap_time_ms);

    in.current_lap_ms = 12345;
    in.timer_running = false;
    in.timer_paused = true;
    history.update(in);
    TEST_ASSERT_TRUE(history.active_frame(frame));
    TEST_ASSERT_EQUAL_UINT32(12345, frame.lap_time_ms);
    TEST_ASSERT_FALSE(frame.timer_running);
    TEST_ASSERT_TRUE(frame.timer_paused);

    in.current_lap_ms = 12545;
    in.timer_running = true;
    in.timer_paused = false;
    history.update(in);
    TEST_ASSERT_TRUE(history.active_frame(frame));
    TEST_ASSERT_EQUAL_UINT32(12545, frame.lap_time_ms);
    TEST_ASSERT_TRUE(frame.timer_running);

    // The final Lap 1 value is repeated three transition cycles total.
    TEST_ASSERT_TRUE(history.take_transition_frame(frame));
    TEST_ASSERT_TRUE(history.take_transition_frame(frame));
    TEST_ASSERT_FALSE(history.take_transition_frame(frame));
    TEST_ASSERT_TRUE(history.next_completed_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(1, frame.lap_number);
    TEST_ASSERT_EQUAL_UINT32(58430, frame.lap_time_ms);
}

void test_lap_history_reset_clears_old_slots(void) {
    ClusterLapHistory history;
    ClusterLapHistoryInput in;
    ClusterLapHistoryFrame frame;
    in.current_lap_number = 1;
    in.timer_running = true;
    history.update(in);
    in.current_lap_number = 2;
    in.completed_lap_count = 1;
    in.last_lap_ms = 58430;
    history.update(in);
    const uint8_t first_session = history.session();

    in = ClusterLapHistoryInput{};
    history.update(in);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(first_session + 1), history.session());
    TEST_ASSERT_FALSE(history.active_frame(frame));
    TEST_ASSERT_FALSE(history.next_completed_frame(frame));
    TEST_ASSERT_TRUE(history.take_transition_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(1, frame.lap_number);
    TEST_ASSERT_FALSE(frame.valid);
    TEST_ASSERT_FALSE(frame.battery_valid);
    TEST_ASSERT_EQUAL_UINT32(0, frame.lap_time_ms);
    TEST_ASSERT_TRUE(history.take_transition_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(2, frame.lap_number);
    TEST_ASSERT_FALSE(frame.valid);

    in.current_lap_number = 1;
    history.update(in);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(first_session + 1), history.session());
    TEST_ASSERT_TRUE(history.active_frame(frame));
    TEST_ASSERT_EQUAL_UINT8(1, frame.lap_number);
    TEST_ASSERT_EQUAL_UINT32(0, frame.lap_time_ms);
}

void test_completed_laps_refresh_each_second(void) {
    const uint8_t counts[] = {1, 7, 99};
    for (const uint8_t count : counts) {
        ClusterLapHistory history;
        ClusterLapHistoryInput in;
        in.current_lap_number = 1;
        history.update(in);
        for (uint8_t lap = 1; lap <= count; ++lap) {
            in.completed_lap_count = lap;
            in.current_lap_number = lap == 99 ? 99 : lap + 1;
            in.last_lap_ms = 50000 + lap;
            history.update(in);
        }
        ClusterLapHistoryFrame frame;
        const uint32_t start = 0xFFFFFF00u;
        TEST_ASSERT_FALSE(history.due_completed_frame(start, frame));
        uint8_t seen[99]{};
        for (uint32_t elapsed = 5; elapsed <= 3000; elapsed += 5) {
            if (!history.due_completed_frame(start + elapsed, frame)) continue;
            TEST_ASSERT_TRUE(frame.completed);
            TEST_ASSERT_EQUAL_UINT32(50000 + frame.lap_number, frame.lap_time_ms);
            ++seen[frame.lap_number - 1];
        }
        for (uint8_t lap = 0; lap < count; ++lap) TEST_ASSERT_EQUAL_UINT8(3, seen[lap]);
        // Reset discards every completed slot, including pending replay credit.
        history.update(ClusterLapHistoryInput{});
        TEST_ASSERT_FALSE(history.due_completed_frame(start + 3010, frame));
        TEST_ASSERT_FALSE(history.due_completed_frame(start + 5010, frame));
    }
}

void test_completed_replay_no_catchup_burst(void) {
    ClusterLapHistory history;
    ClusterLapHistoryInput in;
    in.current_lap_number = 2;
    in.completed_lap_count = 1;
    in.last_lap_ms = 58430;
    history.update(in);
    ClusterLapHistoryFrame frame;
    TEST_ASSERT_FALSE(history.due_completed_frame(100, frame));
    TEST_ASSERT_TRUE(history.due_completed_frame(10100, frame));
    TEST_ASSERT_FALSE(history.due_completed_frame(10100, frame));
    TEST_ASSERT_FALSE(history.due_completed_frame(10105, frame));
}

void test_encode_reset_report(void) {
    // 모든 ESP32 노드 공통 배치. 업타임은 LE 32비트, 리셋 횟수는 255에서 포화.
    uint8_t d[8];
    encode_reset_report(9u, 15u, 0x12345678u, 3u, 42u, d);
    TEST_ASSERT_EQUAL_UINT8(9, d[0]);
    TEST_ASSERT_EQUAL_UINT8(15, d[1]);
    TEST_ASSERT_EQUAL_UINT32(0x12345678u,
        (uint32_t)d[2] | ((uint32_t)d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24));
    TEST_ASSERT_EQUAL_UINT8(3, d[6]);
    TEST_ASSERT_EQUAL_UINT8(42, d[7]);
    encode_reset_report(1u, 1u, 0u, 1000u, 0u, d);
    TEST_ASSERT_EQUAL_UINT8(255, d[6]);
    TEST_ASSERT_EQUAL_UINT32(0x1CFDFFC0u, CAN_ID_CLUSTER_RESET_REPORT);
}

void setUp(void) {}
void test_em_voltage_decode(void) {
    uint8_t data[8] = {0xD7, 0x12, 0x1D, 0xFA, 0x3E, 0x05, 0x4E, 0x0C};
    auto v = decode_em_voltages(data);
    TEST_ASSERT_EQUAL_INT16(4823, v.hv_decivolts);
    TEST_ASSERT_EQUAL_INT16(1342, v.lv_centivolts);
    data[0] = 0; data[1] = 0x80;
    data[4] = 0xFF; data[5] = 0xFF;
    v = decode_em_voltages(data);
    TEST_ASSERT_EQUAL_INT16(-32768, v.hv_decivolts);
    TEST_ASSERT_EQUAL_INT16(-1, v.lv_centivolts);
}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_em_voltage_decode);
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
    RUN_TEST(test_decode_vcu_vehicle_speed_invalid_keeps_value);
    RUN_TEST(test_decode_vcu_vehicle_speed_zero_valid);
    RUN_TEST(test_decode_vcu_vehicle_speed_max_value);
    RUN_TEST(test_decode_vcu_cluster_status_paddock_feedback);
    RUN_TEST(test_encode_config_flags);
    RUN_TEST(test_encode_paddock_bit);
    RUN_TEST(test_encode_regen_level_as_vcu_boolean);
    RUN_TEST(test_ezkontrol_handshake_probe_detection);
    RUN_TEST(test_ezkontrol_handshake_ack_detection);
    RUN_TEST(test_decode_motor_target_current);
    RUN_TEST(test_encode_cluster_bms_status);
    RUN_TEST(test_encode_cluster_bms_detail);
    RUN_TEST(test_encode_cluster_gnss_position);
    RUN_TEST(test_encode_cluster_gnss_rtk_status);
    RUN_TEST(test_encode_cluster_gnss_speed);
    RUN_TEST(test_encode_cluster_gnss_speed_invalid_clears_value);
    RUN_TEST(test_encode_cluster_gnss_speed_saturates);
    RUN_TEST(test_encode_cluster_lap_time);
    RUN_TEST(test_encode_cluster_lap_status);
    RUN_TEST(test_encode_cluster_lap_history);
    RUN_TEST(test_encode_cluster_lap_battery);
    RUN_TEST(test_lap_history_crossing_and_pause_resume);
    RUN_TEST(test_lap_history_reset_clears_old_slots);
    RUN_TEST(test_completed_laps_refresh_each_second);
    RUN_TEST(test_completed_replay_no_catchup_burst);
    RUN_TEST(test_encode_reset_report);
    return UNITY_END();
}
