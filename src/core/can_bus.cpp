// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/can_bus.h"
#include <Arduino.h>
#include "driver/twai.h"
#include "can_protocol.h"
#include "state.h"
#include "core/gps_laptimer.h"
#include "core/ntrip.h"

namespace can_bus {

void begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_18, GPIO_NUM_17, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
}

namespace {
    uint16_t u16le(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

    constexpr uint32_t BMS_CAN_STALE_MS = 5000;
    constexpr uint32_t GPS_CAN_STALE_MS = 3000;
    constexpr uint32_t RTCM_CAN_FRESH_MS = 5000;

    float absf(float v) { return v < 0.0f ? -v : v; }

    void update_display_rpm() {
        const float left = absf(state.speed_rpm_l);
        const float right = absf(state.speed_rpm_r);
        if (state.controller_l_seen && state.controller_r_seen) {
            state.speed_rpm = (left + right) * 0.5f;
        } else if (state.controller_l_seen) {
            state.speed_rpm = left;
        } else if (state.controller_r_seen) {
            state.speed_rpm = right;
        }
    }

    // Part I: bytes 0-1 voltage, 2-3 bus current, 4-5 phase current (unused), 6-7 speed.
    void decode_fb1(const uint8_t *d, float &voltage, float &current, float &speed) {
        voltage = raw_to_voltage(u16le(d + 0));
        current = raw_to_current(u16le(d + 2));
        speed   = (float)raw_to_speed(u16le(d + 6));
    }

    // Part II: byte 0 controller temp, 1 motor temp, 2 status, 3-5 error bitmaps.
    void decode_fb2(const uint8_t *d, int &ctrl_temp, int &motor_temp,
                     uint8_t &status, uint8_t &err1, uint8_t &err2, uint8_t &err3) {
        ctrl_temp  = raw_to_temp(d[0]);
        motor_temp = raw_to_temp(d[1]);
        status = d[2];
        err1 = d[3];
        err2 = d[4];
        err3 = d[5];
    }

    void decode_vcu_cluster_status(const uint8_t *d, uint32_t now) {
        const VcuClusterStatus status = ::decode_vcu_cluster_status(d);
        if (status.gear_valid) {
            state.gear = status.gear;
            state.gear_from_can = true;
        }

        state.brake = status.brake;
        state.hv_active = status.hv_active;
        state.paddock_active = status.paddock_active;
        state.throttle_valid = status.throttle_valid;
        state.throttle_pct = status.throttle_valid ? (float)status.throttle_pct : 0.0f;
        state.throttle_last_rx_ms = state.throttle_valid ? now : 0;

        if (status.soc_valid) {
            state.soc = (float)status.soc_pct * 0.01f;
            state.soc_valid = true;
        } else {
            // Do not clear SOC here: a direct BLE BMS reader may be the source.
        }
    }

    void transmit_ext(uint32_t id, const uint8_t data[8]) {
        twai_message_t m = {};
        m.identifier = id;
        m.extd = 1;
        m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        twai_transmit(&m, pdMS_TO_TICKS(5));
    }

    uint8_t soc_percent() {
        if (!state.soc_valid) return 0;
        int pct = (int)(state.soc * 100.0f + 0.5f);
        if (pct < 0) return 0;
        if (pct > 100) return 100;
        return (uint8_t)pct;
    }

    ClusterBmsStatus snapshot_bms_status(uint32_t now) {
        ClusterBmsStatus bms;
        const bool fresh = state.bms_last_rx_ms != 0 &&
                           (now - state.bms_last_rx_ms) <= BMS_CAN_STALE_MS;
        bms.valid = state.bms_ble_connected && fresh && state.soc_valid;
        bms.ble_connected = state.bms_ble_connected;
        bms.soc_pct = soc_percent();
        bms.pack_voltage_v = state.bms_pack_voltage;
        bms.current_a = state.bms_current;
        bms.temp_c = state.bms_temp_c;
        bms.remaining_mah = state.bms_remaining_mah;
        bms.soh_pct = state.bms_soh;
        bms.cycles = state.bms_cycles;
        return bms;
    }

    uint16_t rtcm_age_dsec(uint32_t now) {
        const uint32_t last_rtcm = ntrip::last_rtcm_ms();
        if (last_rtcm == 0) return 0xFFFF;
        const uint32_t age_ms = now - last_rtcm;
        const uint32_t dsec = (age_ms + 50UL) / 100UL;
        return dsec > 0xFFFFUL ? 0xFFFF : (uint16_t)dsec;
    }

    uint8_t rtk_state_from_fix_quality(uint8_t quality) {
        if (quality == 4) return 2;
        if (quality == 5) return 1;
        return 0;
    }

    ClusterGnssRtkStatus snapshot_gnss_rtk_status(uint32_t now) {
        ClusterGnssRtkStatus status;
        status.gps_data_fresh = state.gps_last_rx_ms != 0 &&
                                (now - state.gps_last_rx_ms) <= GPS_CAN_STALE_MS;
        status.gps_fix_valid = status.gps_data_fresh && state.gps_fix_ok;
        status.ntrip_connected = ntrip::connected();

        const uint32_t last_rtcm = ntrip::last_rtcm_ms();
        status.rtcm_fresh = last_rtcm != 0 &&
                            (now - last_rtcm) <= RTCM_CAN_FRESH_MS;
        status.fix_quality = status.gps_data_fresh ? gps_laptimer::fix_quality() : 0;
        status.rtk_state = status.gps_data_fresh
            ? rtk_state_from_fix_quality(status.fix_quality)
            : 0;
        status.satellites = status.gps_data_fresh ? gps_laptimer::satellites() : 0;
        status.hdop = status.gps_data_fresh ? gps_laptimer::hdop() : 0.0f;
        status.rtcm_age_dsec = rtcm_age_dsec(now);
        return status;
    }
}

void poll_rx() {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd || m.data_length_code < 8) continue;
        const uint32_t now = millis();
        if ((m.identifier == CAN_ID_FB1_L || m.identifier == CAN_ID_FB1_R) &&
            is_ezkontrol_handshake_probe(m.data)) {
            continue;
        }
        switch (m.identifier) {
            case CAN_ID_TORQUE_L:
                if (!is_ezkontrol_handshake_ack(m.data)) {
                    state.torque_cmd_l_a = decode_motor_target_current_a(m.data);
                    state.torque_cmd_l_last_ms = now;
                }
                break;
            case CAN_ID_TORQUE_R:
                if (!is_ezkontrol_handshake_ack(m.data)) {
                    state.torque_cmd_r_a = decode_motor_target_current_a(m.data);
                    state.torque_cmd_r_last_ms = now;
                }
                break;
            case CAN_ID_FB1_L:
                decode_fb1(m.data, state.bus_voltage, state.bus_current, state.speed_rpm_l);
                state.controller_l_seen = true;
                state.controller_l_fb1_last_ms = now;
                update_display_rpm();
                break;
            case CAN_ID_FB1_R:
                decode_fb1(m.data, state.bus_voltage_r, state.bus_current_r, state.speed_rpm_r);
                state.controller_r_seen = true;
                state.controller_r_fb1_last_ms = now;
                update_display_rpm();
                break;
            case CAN_ID_FB2_L:
                decode_fb2(m.data, state.controller_temp, state.motor_temp,
                           state.controller_status, state.error1, state.error2, state.error3);
                state.controller_l_fb2_last_ms = now;
                break;
            case CAN_ID_FB2_R:
                decode_fb2(m.data, state.controller_temp_r, state.motor_temp_r,
                           state.controller_status_r, state.error1_r, state.error2_r, state.error3_r);
                state.controller_r_fb2_last_ms = now;
                break;
            case CAN_ID_VCU_CLUSTER_STATUS:
                state.vcu_cluster_status_last_ms = now;
                decode_vcu_cluster_status(m.data, now);
                break;
            case CAN_ID_VCU_VEHICLE_SPEED:
                decode_vcu_vehicle_speed(m.data, state.vehicle_speed_kph, state.vehicle_speed_valid);
                state.vehicle_speed_last_rx_ms = now;
                break;
            default:
                break;
        }
    }
}

void send_command(const ClusterCommand &cmd) {
    uint8_t data[8];
    encode_cluster_command(cmd, data);
    transmit_ext(CAN_ID_CLUSTER_CMD, data);
}

void send_bms_status() {
    static uint8_t life = 0;
    uint8_t data[8];
    const ClusterBmsStatus bms = snapshot_bms_status(millis());

    encode_cluster_bms_status(bms, life, data);
    transmit_ext(CAN_ID_CLUSTER_BMS_STATUS, data);

    encode_cluster_bms_detail(bms, life, data);
    transmit_ext(CAN_ID_CLUSTER_BMS_DETAIL, data);

    ++life;
}

void send_gnss_position() {
    uint8_t data[8];
    ClusterGnssPosition pos;
    pos.latitude_deg = state.gps_latitude;
    pos.longitude_deg = state.gps_longitude;
    encode_cluster_gnss_position(pos, data);
    transmit_ext(CAN_ID_CLUSTER_GNSS_POSITION, data);
}

void send_gnss_rtk_status() {
    uint8_t data[8];
    encode_cluster_gnss_rtk_status(snapshot_gnss_rtk_status(millis()), data);
    transmit_ext(CAN_ID_CLUSTER_GNSS_RTK_STATUS, data);
}

void send_lap_time() {
    uint8_t data[8];
    encode_cluster_lap_time(state.current_lap_ms, state.last_lap_ms, data);
    transmit_ext(CAN_ID_CLUSTER_LAP_TIME, data);
}

void send_lap_status(bool timer_running) {
    static uint8_t life = 0;
    uint8_t data[8];
    ClusterLapStatus lap;
    lap.best_lap_ms = state.best_lap_ms;
    lap.lap_count = state.lap_count;
    lap.best_lap_count = state.best_lap_count;
    lap.timer_running = timer_running;
    lap.life = life++;
    encode_cluster_lap_status(lap, data);
    transmit_ext(CAN_ID_CLUSTER_LAP_STATUS, data);
}

} // namespace can_bus
