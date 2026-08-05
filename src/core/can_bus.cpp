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

    void decode_vcu_cluster_status(const uint8_t *d) {
        if (d[0] <= 3) {
            state.gear = d[0];
            state.gear_from_can = true;
        }

        state.brake = (d[1] & 0x01) != 0;
        state.hv_active = (d[1] & 0x02) != 0;

        if (d[1] & 0x04) {
            uint8_t pct = d[2];
            if (pct > 100) pct = 100;
            state.soc = (float)pct * 0.01f;
            state.soc_valid = true;
        } else {
            // Do not clear SOC here: a direct BLE BMS reader may be the source.
        }
    }

    void decode_vcu_vehicle_speed(const uint8_t *d) {
        const bool valid = d[2] == 1;
        state.vehicle_speed_valid = valid;
        state.vehicle_speed_kph = valid ? (float)u16le(d + 0) * 0.1f : 0.0f;
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
}

void poll_rx() {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd || m.data_length_code < 8) continue;
        const uint32_t now = millis();
        switch (m.identifier) {
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
                decode_vcu_cluster_status(m.data);
                break;
            case CAN_ID_VCU_VEHICLE_SPEED:
                decode_vcu_vehicle_speed(m.data);
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

} // namespace can_bus
