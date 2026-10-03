// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/can_bus.h"
#include <Arduino.h>
#include "driver/twai.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "can_protocol.h"
#include "state.h"
#include "core/gps_laptimer.h"
#include "core/ntrip.h"
#include "core/board_pins.h"
#include "modules/drivetrain_monitor.h"

namespace can_bus {

namespace {
    // Reset cause of this boot. The count survives every reset except power
    // loss (RTC no-init memory), so it separates brownout/crash loops from a
    // fresh key-on.
    constexpr uint32_t RESET_MAGIC = 0x52535431u; // "RST1"
    RTC_NOINIT_ATTR uint32_t g_reset_magic;
    RTC_NOINIT_ATTR uint32_t g_reset_count;
    uint8_t g_reset_reason = 0;
    uint8_t g_reset_rom_reason = 0;

    // Bus-off is handled by reinstalling the driver, as in the VCU (vcu
    // 0c27448): in ESP-IDF 4.4.x twai_initiate_recovery() zeroes tx_msg_count
    // while a frame can still sit in the TX buffer, and the later TX interrupt
    // trips assert(tx_msg_count >= 0) in twai.c. Without any recovery the
    // Cluster stays silent after a bus-off until it is power-cycled. While
    // g_twai_offline is set every transmit returns at once.
    volatile bool g_twai_offline = false;
    uint32_t g_twai_offline_ms = 0U;
    uint32_t g_twai_bus_off_count = 0U;
    constexpr uint32_t TWAI_REINSTALL_SETTLE_MS = 20U; // > every transmit wait (5 ms)

    void note_reset() {
        g_reset_reason = static_cast<uint8_t>(esp_reset_reason());
        g_reset_rom_reason = static_cast<uint8_t>(esp_rom_get_reset_reason(0));
        if (g_reset_reason == ESP_RST_POWERON || g_reset_magic != RESET_MAGIC) {
            g_reset_magic = RESET_MAGIC;
            g_reset_count = 0U;
        } else {
            ++g_reset_count;
        }
        Serial.printf("[BOOT] reset reason=%u rom=%u count=%lu\n",
                      g_reset_reason, g_reset_rom_reason,
                      static_cast<unsigned long>(g_reset_count));
    }
}

namespace {
    esp_err_t install_twai() {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
            static_cast<gpio_num_t>(board_pins::CAN_TX),
            static_cast<gpio_num_t>(board_pins::CAN_RX),
            TWAI_MODE_NORMAL);
        // Absorb telemetry bursts while the shared task renders/transfers an LCD frame.
        g.rx_queue_len = 64;
        twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
        twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        const esp_err_t err = twai_driver_install(&g, &t, &f);
        return err == ESP_OK ? twai_start() : err;
    }

    // Bus-off: stop transmitting, let any in-flight transmit time out, then
    // reinstall the driver. A fresh install also discards stale queued frames.
    void recover_bus_off(uint32_t now) {
        twai_status_info_t status;
        if (!g_twai_offline && twai_get_status_info(&status) == ESP_OK &&
            status.state == TWAI_STATE_BUS_OFF) {
            g_twai_offline = true;
            g_twai_offline_ms = now;
            ++g_twai_bus_off_count;
            Serial.printf("[CAN] TWAI bus-off #%lu, reinstalling driver\n",
                          static_cast<unsigned long>(g_twai_bus_off_count));
        }
        if (!g_twai_offline || now - g_twai_offline_ms < TWAI_REINSTALL_SETTLE_MS) return;

        const esp_err_t uninstall_result = twai_driver_uninstall();
        const esp_err_t install_result = install_twai();
        if (install_result == ESP_OK) {
            g_twai_offline = false;
            Serial.println("[CAN] TWAI reinstalled");
        } else {
            // Retry after another settle period rather than spin.
            g_twai_offline_ms = now;
            Serial.printf("[CAN] TWAI reinstall failed: uninstall %d install %d\n",
                          static_cast<int>(uninstall_result),
                          static_cast<int>(install_result));
        }
    }
}

void begin() {
    install_twai();
    note_reset();
}

namespace {
    uint16_t u16le(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

    constexpr uint32_t BMS_CAN_STALE_MS = 5000;
    constexpr uint32_t GPS_CAN_STALE_MS = 3000;
    constexpr uint32_t RTCM_CAN_FRESH_MS = 5000;
    DrivetrainMonitor drivetrain_monitor;

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

    void update_drivetrain_monitor(DrivetrainSide side, float rpm, uint32_t now) {
        const DrivetrainMonitorEvent event =
            drivetrain_monitor.receive(side, static_cast<int>(rpm), now);
        const DrivetrainMonitorStatus &status = drivetrain_monitor.status();
        state.drivetrain_fault_l = static_cast<uint8_t>(status.left);
        state.drivetrain_fault_r = static_cast<uint8_t>(status.right);
        state.drivetrain_dropouts_l = status.left_dropouts;
        state.drivetrain_dropouts_r = status.right_dropouts;
        state.drivetrain_last_event_ms_l = status.left_last_event_ms;
        state.drivetrain_last_event_ms_r = status.right_last_event_ms;
        if (!event.triggered) return;

        state.drivetrain_last_fault = static_cast<uint8_t>(event.fault);
        state.drivetrain_last_side = static_cast<uint8_t>(event.side);
        state.drivetrain_warning_started_ms = event.ms;
        ++state.drivetrain_warning_sequence;
    }

    // Part I: bytes 0-1 voltage, 2-3 bus current, 4-5 phase current, 6-7 speed.
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

    void transmit_ext(uint32_t id, const uint8_t data[8], uint32_t wait_ms = 5) {
        twai_message_t m = {};
        m.identifier = id;
        m.extd = 1;
        m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        if (g_twai_offline) return;
        twai_transmit(&m, pdMS_TO_TICKS(wait_ms));
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
        // Written by the NimBLE notify callback; may be newer than `now`.
        const bool fresh = state.bms_last_rx_ms != 0 &&
                           (int32_t)(now - state.bms_last_rx_ms) <= (int32_t)BMS_CAN_STALE_MS;
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
        // Written by the NTRIP task on core 0; may be newer than `now`.
        const uint32_t age_ms = (int32_t)(now - last_rtcm) < 0 ? 0U : now - last_rtcm;
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
                            (int32_t)(now - last_rtcm) <= (int32_t)RTCM_CAN_FRESH_MS;
        status.fix_quality = status.gps_data_fresh ? gps_laptimer::fix_quality() : 0;
        status.rtk_state = status.gps_data_fresh
            ? rtk_state_from_fix_quality(status.fix_quality)
            : 0;
        status.satellites = status.gps_data_fresh ? gps_laptimer::satellites() : 0;
        status.hdop = status.gps_data_fresh ? gps_laptimer::hdop() : 0.0f;
        status.rtcm_age_dsec = rtcm_age_dsec(now);
        return status;
    }

    ClusterGnssSpeed snapshot_gnss_speed(uint32_t now, uint8_t life) {
        ClusterGnssSpeed speed;
        const bool fresh = state.gps_rmc_last_rx_ms != 0 &&
                           (now - state.gps_rmc_last_rx_ms) <= GPS_CAN_STALE_MS;
        speed.rmc_fresh = fresh;
        speed.gps_fix_valid = fresh && state.gps_fix_ok;
        speed.speed_valid = speed.gps_fix_valid && state.gps_ground_speed_valid;
        speed.speed_kph = speed.speed_valid ? state.gps_ground_speed_kph : 0.0f;
        speed.fix_quality = fresh ? gps_laptimer::fix_quality() : 0;
        speed.rtk_state = fresh ? rtk_state_from_fix_quality(speed.fix_quality) : 0;
        if (state.gps_rmc_last_rx_ms != 0) {
            const uint32_t age_dsec = ((now - state.gps_rmc_last_rx_ms) + 50UL) / 100UL;
            speed.rmc_age_dsec = age_dsec > 0xFFFFUL ? 0xFFFF : (uint16_t)age_dsec;
        }
        speed.life = life;
        return speed;
    }
}

void poll_rx() {
    recover_bus_off(millis());
    if (g_twai_offline) return;

    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd || m.rtr || m.data_length_code != 8) continue;
        const uint32_t now = millis();
        if (state.car_check.receive(m.identifier, m.data, m.data_length_code, m.extd, m.rtr, now)) continue;
        if ((m.identifier == CAN_ID_FB1_L || m.identifier == CAN_ID_FB1_R) &&
            is_ezkontrol_handshake_probe(m.data)) {
            continue;
        }
        switch (m.identifier) {
            case CAN_ID_EM_RECORD: {
                const EmVoltages volts = decode_em_voltages(m.data);
                state.em_hv_decivolts = volts.hv_decivolts;
                state.em_lv_centivolts = volts.lv_centivolts;
                const uint16_t amps = u16le(m.data + 2), temp = u16le(m.data + 6);
                state.em_current_deciamps = amps < 32768 ? amps : static_cast<int32_t>(amps)-65536;
                state.em_cpu_centidegrees = temp < 32768 ? temp : static_cast<int32_t>(temp)-65536;
                state.em_record_last_ms = now;
                state.em_record_seen = true;
                break;
            }
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
                state.phase_current = raw_to_current(u16le(m.data + 4));
                decode_fb1(m.data, state.bus_voltage, state.bus_current, state.speed_rpm_l);
                state.controller_l_seen = true;
                state.controller_l_fb1_last_ms = now;
                update_drivetrain_monitor(DrivetrainSide::Left, state.speed_rpm_l, now);
                update_display_rpm();
                break;
            case CAN_ID_FB1_R:
                state.phase_current_r = raw_to_current(u16le(m.data + 4));
                decode_fb1(m.data, state.bus_voltage_r, state.bus_current_r, state.speed_rpm_r);
                state.controller_r_seen = true;
                state.controller_r_fb1_last_ms = now;
                update_drivetrain_monitor(DrivetrainSide::Right, state.speed_rpm_r, now);
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
            {
                float kph = 0.0f;
                bool valid = false;
                decode_vcu_vehicle_speed(m.data, kph, valid);
                // invalid 프레임은 직전 valid 값을 유지한다. 구버전 VCU는 invalid일 때
                // 속도를 0으로 보내므로 그대로 쓰면 표시가 0으로 튄다.
                if (valid) {
                    state.wss_kph = kph;
                    state.wss_last_valid_ms = now;
                }
                state.wss_valid = valid;
                state.vehicle_speed_kph = state.wss_kph;
                state.vehicle_speed_valid = valid;
                state.vehicle_speed_last_rx_ms = now;
            }
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

void send_gnss_speed() {
    static uint8_t life = 0;
    uint8_t data[8];
    encode_cluster_gnss_speed(snapshot_gnss_speed(millis(), life++), data);
    transmit_ext(CAN_ID_CLUSTER_GNSS_SPEED, data);
}

void send_lap_time() {
    uint8_t data[8];
    encode_cluster_lap_time(state.current_lap_ms, state.last_lap_ms, data);
    transmit_ext(CAN_ID_CLUSTER_LAP_TIME, data);
}

void send_reset_report() {
    static uint8_t life = 0;
    uint8_t data[8];
    encode_reset_report(g_reset_reason, g_reset_rom_reason, millis(),
                        g_reset_count, life++, data);
    transmit_ext(CAN_ID_CLUSTER_RESET_REPORT, data);
}

void send_loop_timing(const uint8_t data[8]) {
    transmit_ext(CAN_ID_CLUSTER_LOOP_TIMING, data);
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

void send_lap_history(uint8_t current_lap_number, bool timer_running,
                      bool timer_paused) {
    static ClusterLapHistory history;
    static uint8_t life = 0;
    static uint32_t active_send_ms = 0;
    const uint32_t now = millis();

    ClusterLapHistoryInput input;
    input.current_lap_number = current_lap_number;
    input.completed_lap_count = state.lap_count;
    input.current_lap_ms = state.current_lap_ms;
    input.last_lap_ms = state.last_lap_ms;
    input.current_battery_used_x10 = state.current_lap_battery_used_x10;
    input.last_battery_used_x10 = state.last_lap_battery_used_x10;
    input.current_battery_valid = state.current_lap_battery_valid;
    input.last_battery_valid = state.last_lap_battery_valid;
    input.timer_running = timer_running;
    input.timer_paused = timer_paused;
    history.update(input);

    auto send = [&](ClusterLapHistoryFrame &frame) {
        uint8_t data[8];
        frame.life = life++;
        encode_cluster_lap_history(frame, data);
        transmit_ext(CAN_ID_CLUSTER_LAP_HISTORY, data, 0);
        encode_cluster_lap_battery(frame, data);
        transmit_ext(CAN_ID_CLUSTER_LAP_BATTERY, data, 0);
    };

    ClusterLapHistoryFrame frame;
    if (now - active_send_ms >= 200) {
        active_send_ms = now;
        if (history.take_transition_frame(frame)) send(frame);
        if (history.active_frame(frame)) send(frame);
    }
    if (history.due_completed_frame(now, frame)) send(frame);
}

} // namespace can_bus
