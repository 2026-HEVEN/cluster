// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/wiring.h"
#include "state.h"
#include "core/can_bus.h"
#include "core/display_blit.h"
#include "core/bms_ble.h"
#include "core/gps_laptimer.h"
#include "core/ntrip.h"
#include "framebuffer.h"
#include "modules/hmi_input.h"
#include "modules/widgets/widget_speed.h"
#include "modules/widgets/widget_battery.h"
#include "modules/widgets/widget_warnings.h"
#include "modules/widgets/widget_gear.h"
#include "modules/widgets/widget_laptime.h"
#include <Arduino.h>
#include <XPT2046_Touchscreen.h>
#include <cstdio>

// [LOCKED] The ONLY translation unit that touches `state`.
ClusterState state;

namespace {
    // Input pins (direct GPIO; if pin count runs short, an io_expander can be
    // reintroduced HERE only, without touching any module).
    constexpr int PIN_PADDOCK = 36; // SVP/GPIO36; external 10k pull-up required on PCB
    constexpr int PIN_TC = 25;
    constexpr int PIN_REGEN_BIT0 = 27; // regen rotary bit 0; ON=LOW
    constexpr int PIN_REGEN_BIT1 = 34; // regen rotary bit 1; GPIO34 needs external 10k pull-up
    constexpr int PIN_DEBUG = 26;
    constexpr int PIN_GPS_LAP_START = 32;     // set GPS lap start
    constexpr int PIN_TOUCH_CS = 23;        // XPT2046 touch chip select; touch toggles vehicle status
    constexpr uint32_t CAN_STARTUP_GRACE_MS = 3000;
    constexpr uint32_t CONTROLLER_FRAME_TIMEOUT_MS = 300;
    constexpr uint32_t VCU_STATUS_TIMEOUT_MS = 300;
    constexpr uint32_t VEHICLE_SPEED_TIMEOUT_MS = 300;
    constexpr uint32_t GPS_SIGNAL_TIMEOUT_MS = 3000;
    constexpr uint32_t GPS_POSITION_CAN_STALE_MS = 3000;
    constexpr uint32_t LAP_NOTICE_MS = 1500;
    constexpr uint32_t NTRIP_START_DELAY_MS = 5000;
    constexpr float WHEEL_DIAMETER_M = 0.4597f;
    constexpr float MOTOR_TO_WHEEL_RATIO = 3.72f;
    constexpr float PI_F = 3.14159265f;
    constexpr int TOUCH_RAW_MID_X = 2048;
    constexpr bool TOUCH_RIGHT_IS_NEXT = true;
    enum DisplayPage : uint8_t { PAGE_MAIN = 0, PAGE_CAR_CHECK = 1, PAGE_WARNING_DETAIL = 2 };
    constexpr uint8_t DISPLAY_PAGE_COUNT = 3;
    FrameBuffer fb;
    DisplayPage display_page = PAGE_MAIN;
    bool status_touch_down = false;
    uint32_t status_touch_last_ms = 0;
    constexpr uint32_t STATUS_TOUCH_DEBOUNCE_MS = 120;
    XPT2046_Touchscreen touch(PIN_TOUCH_CS);
    bool gps_lap_start_button_down = false;
    uint32_t gps_lap_start_button_last_ms = 0;
    const char *lap_notice_label = nullptr;
    uint32_t lap_notice_until_ms = 0;
    bool gps_fix_was_ok = false;
    bool ntrip_started = false;
    uint32_t last_gnss_position_seq_sent = 0;

    int gear_code(uint8_t gear) { return gear <= 3 ? gear : 0; }

    float absf(float v) { return v < 0.0f ? -v : v; }

    float motor_rpm_to_kph(float motor_rpm) {
        return (absf(motor_rpm) / MOTOR_TO_WHEEL_RATIO) *
               (PI_F * WHEEL_DIAMETER_M) * 0.06f;
    }

    bool frame_fresh(uint32_t last_ms, uint32_t now, uint32_t timeout_ms) {
        return last_ms != 0 && (now - last_ms) <= timeout_ms;
    }

    bool frame_stale(uint32_t last_ms, uint32_t now, uint32_t timeout_ms) {
        return !frame_fresh(last_ms, now, timeout_ms);
    }

    bool controller_fault_active() {
        return state.error1 || state.error2 || state.error3 ||
               state.error1_r || state.error2_r || state.error3_r;
    }

    bool controller_feedback_stale(uint32_t now) {
        if (now < CAN_STARTUP_GRACE_MS) return false;
        return frame_stale(state.controller_l_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS) ||
               frame_stale(state.controller_l_fb2_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS) ||
               frame_stale(state.controller_r_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS) ||
               frame_stale(state.controller_r_fb2_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS);
    }

    bool vcu_status_stale(uint32_t now) {
        if (now < CAN_STARTUP_GRACE_MS) return false;
        return frame_stale(state.vcu_cluster_status_last_ms, now, VCU_STATUS_TIMEOUT_MS);
    }

    bool can_link_warning_active(uint32_t now) {
        return controller_feedback_stale(now) || vcu_status_stale(now);
    }

    bool warning_active() {
        const uint32_t now = millis();
        return controller_fault_active() || can_link_warning_active(now);
    }

    void add_warning(const char *labels[], int &count, const char *label) {
        if (count < 48) labels[count++] = label;
    }

    bool gps_signal_fresh(uint32_t now) {
        return state.gps_last_rx_ms != 0 &&
               (now - state.gps_last_rx_ms) <= GPS_SIGNAL_TIMEOUT_MS;
    }

    void show_lap_notice(const char *label, uint32_t now) {
        lap_notice_label = label;
        lap_notice_until_ms = now + LAP_NOTICE_MS;
    }

    void gps_fix_feedback_update() {
        const bool fix_ok = state.gps_fix_ok;
        if (fix_ok && !gps_fix_was_ok) {
            show_lap_notice("GPS FIX ON", millis());
        }
        gps_fix_was_ok = fix_ok;
    }

    void page_next() {
        display_page = (DisplayPage)(((uint8_t)display_page + 1) % DISPLAY_PAGE_COUNT);
    }

    void page_prev() {
        display_page = display_page == PAGE_MAIN ? PAGE_WARNING_DETAIL
                                                 : (DisplayPage)((uint8_t)display_page - 1);
    }

    const char *fresh_label(uint32_t last_ms, uint32_t now, uint32_t timeout_ms) {
        if (last_ms == 0) return "WAIT";
        return frame_fresh(last_ms, now, timeout_ms) ? "OK" : "ERR";
    }

    const char *dual_fresh_label(uint32_t last_a, uint32_t last_b,
                                 uint32_t now, uint32_t timeout_ms) {
        if (last_a == 0 || last_b == 0) return "WAIT";
        return frame_fresh(last_a, now, timeout_ms) &&
               frame_fresh(last_b, now, timeout_ms) ? "OK" : "ERR";
    }

    const char *on_off(bool value) {
        return value ? "ON" : "OFF";
    }

    const char *motor_heat_label(uint8_t err1) {
        return (err1 & (1u << 5)) ? "HOT" : "OK";
    }

    const char *ctrl_heat_label(uint8_t err1) {
        return (err1 & (1u << 4)) ? "HOT" : "OK";
    }

    const char *volt_label(uint8_t err1) {
        if (err1 & (1u << 2)) return "OVER";
        if (err1 & (1u << 3)) return "LOW";
        return "OK";
    }

    const uint8_t *status_glyph(char c) {
        static const uint8_t glyph_b[7] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
        static const uint8_t glyph_s[7] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
        if (c == 'B') return glyph_b;
        if (c == 'S') return glyph_s;
        return font_glyph(c);
    }

    void status_text(int x, int y, const char *text, int scale) {
        if (scale < 1) scale = 1;
        int cx = x;
        for (const char *p = text; *p; ++p) {
            const uint8_t *g = status_glyph(*p);
            if (g) {
                for (int r = 0; r < 7; ++r) {
                    for (int c = 0; c < 5; ++c) {
                        if (g[r] & (0x10 >> c)) {
                            fb_rect(fb, cx + c * scale, y + r * scale,
                                    scale, scale, true, true);
                        }
                    }
                }
            }
            cx += 6 * scale;
        }
    }

    void status_line(int &y, const char *text, int scale) {
        status_text(8, y, text, scale);
        y += scale * 8 + 1;
    }

    bool side_fault(uint8_t err1, uint8_t err2, uint8_t err3) {
        return err1 || err2 || err3;
    }

    const char *fault_label(uint8_t err1, uint8_t err2, uint8_t err3) {
        return side_fault(err1, err2, err3) ? "FAULT" : "OK";
    }

    void draw_side_wait(int &y, const char *side, const char *label) {
        char buf[48];

        std::snprintf(buf, sizeof(buf), "%s %s", side, label);
        status_line(y, buf, 2);
        status_line(y, "MTR ---C WAIT", 2);
        status_line(y, "CTRL ---C WAIT", 2);
        status_line(y, "VOLT ---.-- WAIT", 2);
    }

    void draw_side_status(int &y, const char *side, const char *link_label,
                          int motor_temp, int ctrl_temp,
                          float voltage, uint8_t err1, uint8_t err2, uint8_t err3) {
        char buf[48];
        if (link_label[0] != 'O' || link_label[1] != 'K' || link_label[2] != '\0') {
            draw_side_wait(y, side, link_label);
            return;
        }

        std::snprintf(buf, sizeof(buf), "%s %s", side, fault_label(err1, err2, err3));
        status_line(y, buf, 2);

        std::snprintf(buf, sizeof(buf), "MTR %03dC %s", motor_temp, motor_heat_label(err1));
        status_line(y, buf, 2);

        std::snprintf(buf, sizeof(buf), "CTRL %03dC %s", ctrl_temp, ctrl_heat_label(err1));
        status_line(y, buf, 2);

        std::snprintf(buf, sizeof(buf), "VOLT %03d.%01d %s",
                      (int)voltage, ((int)(voltage * 10.0f)) % 10, volt_label(err1));
        status_line(y, buf, 2);
    }

    void draw_vehicle_status() {
        const uint32_t now = millis();
        char buf[48];
        const char *left_can_label = dual_fresh_label(state.controller_l_fb1_last_ms,
                                                      state.controller_l_fb2_last_ms,
                                                      now, CONTROLLER_FRAME_TIMEOUT_MS);
        const char *right_can_label = dual_fresh_label(state.controller_r_fb1_last_ms,
                                                       state.controller_r_fb2_last_ms,
                                                       now, CONTROLLER_FRAME_TIMEOUT_MS);

        fb_text(fb, 8, 4, "CAR CHECK", 3);

        int y = 31;
        std::snprintf(buf, sizeof(buf), "CAN L %s R %s",
                      left_can_label, right_can_label);
        status_line(y, buf, 2);

        std::snprintf(buf, sizeof(buf), "VCU %s HV %s",
                      fresh_label(state.vcu_cluster_status_last_ms, now, VCU_STATUS_TIMEOUT_MS),
                      on_off(state.hv_active));
        status_line(y, buf, 2);

        const bool bms_ok = state.bms_ble_connected && state.bms_last_rx_ms != 0;
        const int soc_pct = state.soc_valid ? (int)(state.soc * 100.0f + 0.5f) : -1;
        if (bms_ok && soc_pct >= 0) {
            const int pack_v = (int)(state.bms_pack_voltage + 0.5f);
            std::snprintf(buf, sizeof(buf), "BMS OK %03d%% %02dV", soc_pct, pack_v);
        } else {
            std::snprintf(buf, sizeof(buf), "BMS WAIT ---");
        }
        status_line(y, buf, 2);

        fb_vline(fb, 194, 39, 48, true);
        const char *gps_status = state.gps_fix_ok ? "GPS OK" :
                                 (state.gps_data_ok ? "GPS SEARCH" : "GPS NO DATA");
        status_text(210, 39, gps_status, 1);
        if (state.gps_fix_ok) {
            std::snprintf(buf, sizeof(buf), "LAT %.5f", state.gps_latitude);
            status_text(202, 60, buf, 1);
            std::snprintf(buf, sizeof(buf), "LON %.5f", state.gps_longitude);
            status_text(202, 73, buf, 1);
        } else {
            status_text(202, 60, "LAT --.-----", 1);
            status_text(202, 73, "LON ---.-----", 1);
        }
        const unsigned lap = state.lap_count > 99 ? 99 : state.lap_count;
        const unsigned minutes = (unsigned)((state.current_lap_ms / 60000UL) % 100UL);
        const unsigned seconds = (unsigned)((state.current_lap_ms / 1000UL) % 60UL);
        const unsigned centis = (unsigned)((state.current_lap_ms / 10UL) % 100UL);
        std::snprintf(buf, sizeof(buf), "LAP %02u", lap);
        status_text(214, 101, buf, 2);
        if (state.gps_fix_ok) {
            std::snprintf(buf, sizeof(buf), "%02u:%02u.%02u",
                          minutes, seconds, centis);
        } else {
            std::snprintf(buf, sizeof(buf), "--:--.--");
        }
        status_text(206, 123, buf, 2);
        status_text(206, 146, gps_laptimer::rtk_status_label(), 1);
        status_text(206, 159, ntrip::status_label(), 1);

        const uint32_t rtcm_ms = ntrip::last_rtcm_ms();
        const char *rtcm_status = rtcm_ms == 0 ? "RTCM WAIT" :
                                  (now - rtcm_ms <= 5000UL ? "RTCM OK" : "RTCM OLD");
        status_text(206, 172, rtcm_status, 1);

        y += 3;
        draw_side_status(y, "LEFT", left_can_label,
                         state.motor_temp, state.controller_temp,
                         state.bus_voltage, state.error1, state.error2, state.error3);

        y += 3;
        draw_side_status(y, "RIGHT", right_can_label,
                         state.motor_temp_r, state.controller_temp_r,
                         state.bus_voltage_r, state.error1_r, state.error2_r, state.error3_r);
    }

    void draw_warning_detail() {
        static const char *const left_labels[3][8] = {
            {"L OVER CURRENT", "L OVER LOAD", "L OVER VOLT", "L LOW VOLT",
             "L CTRL HOT", "L MOTOR HOT", "L MOTOR STALL", "L MOTOR PHASE"},
            {"L MOTOR SENSOR", "L AUX SENSOR", "L ENCODER ALIGN", "L RUNAWAY",
             "L MAIN ACCEL", "L AUX ACCEL", "L PRECHARGE", "L DC CONTACTOR"},
            {"L POWER VALVE", "L CURRENT SENSOR", "L AUTO TUNE", "L RS485",
             "L CAN", "L SOFTWARE", nullptr, nullptr}
        };
        static const char *const right_labels[3][8] = {
            {"R OVER CURRENT", "R OVER LOAD", "R OVER VOLT", "R LOW VOLT",
             "R CTRL HOT", "R MOTOR HOT", "R MOTOR STALL", "R MOTOR PHASE"},
            {"R MOTOR SENSOR", "R AUX SENSOR", "R ENCODER ALIGN", "R RUNAWAY",
             "R MAIN ACCEL", "R AUX ACCEL", "R PRECHARGE", "R DC CONTACTOR"},
            {"R POWER VALVE", "R CURRENT SENSOR", "R AUTO TUNE", "R RS485",
             "R CAN", "R SOFTWARE", nullptr, nullptr}
        };
        const uint8_t left_errors[3] = {state.error1, state.error2, state.error3};
        const uint8_t right_errors[3] = {state.error1_r, state.error2_r, state.error3_r};
        const char *labels[48];
        int count = 0;
        const uint32_t now = millis();

        for (int group = 0; group < 3; ++group) {
            for (int bit = 0; bit < 8; ++bit) {
                if ((left_errors[group] & (1u << bit)) && left_labels[group][bit]) {
                    add_warning(labels, count, left_labels[group][bit]);
                }
                if ((right_errors[group] & (1u << bit)) && right_labels[group][bit]) {
                    add_warning(labels, count, right_labels[group][bit]);
                }
            }
        }
        if (now >= CAN_STARTUP_GRACE_MS) {
            if (frame_stale(state.controller_l_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS) ||
                frame_stale(state.controller_l_fb2_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS)) {
                add_warning(labels, count, "L CAN TIMEOUT");
            }
            if (frame_stale(state.controller_r_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS) ||
                frame_stale(state.controller_r_fb2_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS)) {
                add_warning(labels, count, "R CAN TIMEOUT");
            }
        }
        if (vcu_status_stale(now)) add_warning(labels, count, "VCU CAN TIMEOUT");

        if (count == 0) {
            fb_text(fb, 40, 14, "WARNING", 5);
            fb_text(fb, 40, 112, "NO ERROR", 5);
            return;
        }

        fb_text(fb, 8, 5, "WARNING", 3);
        const int scale = count <= 6 ? 3 : (count <= 22 ? 2 : 1);
        const int step = scale * 8 + 1;
        const bool two_columns = count > 6;
        const int rows_per_column = two_columns ? (count + 1) / 2 : count;
        int y = scale == 1 ? 25 : 38;
        if (!two_columns && count <= 3) y = 76;

        for (int i = 0; i < count; ++i) {
            const int column = two_columns ? i / rows_per_column : 0;
            const int row = two_columns ? i % rows_per_column : i;
            fb_text(fb, column == 0 ? 8 : 164, y + row * step, labels[i], scale);
        }
    }

    uint8_t read_regen_level() {
        const uint8_t bit0 = digitalRead(PIN_REGEN_BIT0) == LOW ? 1u : 0u;
        const uint8_t bit1 = digitalRead(PIN_REGEN_BIT1) == LOW ? 2u : 0u;
        return (uint8_t)(bit0 | bit1);
    }

    void status_touch_update() {
        const bool down = touch.touched();
        const uint32_t now = millis();
        if (down != status_touch_down && now - status_touch_last_ms >= STATUS_TOUCH_DEBOUNCE_MS) {
            status_touch_down = down;
            status_touch_last_ms = now;
            if (down) {
                TS_Point p = touch.getPoint();
                const bool right_side = p.x < TOUCH_RAW_MID_X;
                if (right_side == TOUCH_RIGHT_IS_NEXT) page_next();
                else page_prev();
            }
        }
    }
    void gps_lap_start_update() {
        const bool down = digitalRead(PIN_GPS_LAP_START) == LOW;
        const uint32_t now = millis();
        if (down != gps_lap_start_button_down && now - gps_lap_start_button_last_ms >= 50) {
            gps_lap_start_button_down = down;
            gps_lap_start_button_last_ms = now;
            if (down) {
                if (warning_active()) {
                    gps_laptimer::stop();
                    show_lap_notice("LAP STOPPED", now);
                } else if (gps_laptimer::start_at_current_fix()) {
                    show_lap_notice("LAP START SET", now);
                } else if (!gps_signal_fresh(now)) {
                    show_lap_notice("NO GPS DATA", now);
                } else {
                    show_lap_notice("NO GPS FIX", now);
                }
            }
        }
    }


    void draw_lap_notice(uint32_t now) {
        if (!lap_notice_label || now > lap_notice_until_ms) return;
        constexpr int box_x = 68;
        constexpr int box_y = 94;
        constexpr int box_w = 184;
        constexpr int box_h = 42;
        const int scale = 2;
        int len = 0;
        while (lap_notice_label[len]) ++len;
        int x = 160 - (len * 6 * scale) / 2;
        if (x < box_x + 8) x = box_x + 8;

        fb_rect(fb, box_x, box_y, box_w, box_h, true, false);
        fb_rect(fb, box_x, box_y, box_w, box_h, false, true);
        fb_text(fb, x, box_y + 14, lap_notice_label, scale);
    }

    void refresh_can_timeouts() {
        const uint32_t now = millis();
        const bool left_speed_fresh =
            frame_fresh(state.controller_l_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS);
        const bool right_speed_fresh =
            frame_fresh(state.controller_r_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS);

        if (left_speed_fresh && right_speed_fresh) {
            state.speed_rpm = (absf(state.speed_rpm_l) + absf(state.speed_rpm_r)) * 0.5f;
        } else if (left_speed_fresh) {
            state.speed_rpm = absf(state.speed_rpm_l);
        } else if (right_speed_fresh) {
            state.speed_rpm = absf(state.speed_rpm_r);
        } else if (now >= CAN_STARTUP_GRACE_MS) {
            state.speed_rpm = 0.0f;
        }

        if (left_speed_fresh || right_speed_fresh) {
            state.vehicle_speed_kph = motor_rpm_to_kph(state.speed_rpm);
            state.vehicle_speed_valid = true;
            state.vehicle_speed_last_rx_ms = now;
        }

        if (frame_stale(state.vehicle_speed_last_rx_ms, now, VEHICLE_SPEED_TIMEOUT_MS)) {
            state.vehicle_speed_kph = 0.0f;
            state.vehicle_speed_valid = false;
        }

        if (state.gear_from_can && vcu_status_stale(now)) {
            state.gear_from_can = false;
            state.brake = false;
            state.hv_active = false;
        }
    }
}

static void hmi_update() {
    refresh_can_timeouts();
    gps_fix_feedback_update();
    gps_lap_start_update();
    status_touch_update();

    HmiSwitches sw;
    sw.paddock       = digitalRead(PIN_PADDOCK) == LOW;
    sw.tc_enabled    = digitalRead(PIN_TC) == LOW;
    const uint8_t regen_level = read_regen_level();
    sw.regen_bit0 = (regen_level & 0x01) != 0;
    sw.regen_bit1 = (regen_level & 0x02) != 0;
    sw.debug_enabled = digitalRead(PIN_DEBUG) == LOW;
    ClusterCommand cmd = hmi_compute(sw);
    if (!state.gear_from_can) {
        state.gear = 0;
    }
    state.paddock = cmd.paddock;
    state.tc_enabled = cmd.tc_enabled;
    state.regen_level = cmd.regen_level;
    state.debug_enabled = cmd.debug_enabled;
    state.reset_req  = false;
    can_bus::send_command(cmd);
}

static void can_rx_update() { can_bus::poll_rx(); }
static void gps_update() { gps_laptimer::poll(); }
static void ntrip_update() {
    const uint32_t now = millis();
    if (!ntrip_started) {
        if (now < NTRIP_START_DELAY_MS) return;
        ntrip::begin();
        ntrip_started = true;
    }
    ntrip::poll();
}
static void bms_update() { bms_ble::poll(); }
static void bms_can_tx_update() { can_bus::send_bms_status(); }
static void gnss_position_can_tx_update() {
    const uint32_t seq = gps_laptimer::position_sequence();
    const uint32_t now = millis();
    const bool fresh_fix = state.gps_fix_ok && state.gps_last_rx_ms != 0 &&
                           (now - state.gps_last_rx_ms) <= GPS_POSITION_CAN_STALE_MS;
    if (seq != last_gnss_position_seq_sent && fresh_fix) {
        can_bus::send_gnss_position();
        last_gnss_position_seq_sent = seq;
    }
}
static void gnss_status_can_tx_update() { can_bus::send_gnss_rtk_status(); }
static void lap_can_tx_update() {
    can_bus::send_lap_time();
    can_bus::send_lap_status(gps_laptimer::timer_running());
}
static void display_update() {
    fb.clear();
    const bool warn = warning_active();
    if (display_page == PAGE_CAR_CHECK) {
        draw_vehicle_status();
    } else if (display_page == PAGE_WARNING_DETAIL) {
        draw_warning_detail();
    } else {
        widget_speed_draw(fb,    10,  18, (int)(state.vehicle_speed_kph + 0.5f));
        widget_warnings_draw(fb, 272,  60, warn, state.hv_active,
                             state.regen_level);
        widget_gear_draw(fb,     270,   8, gear_code(state.gear));
        const int soc_pct = state.soc_valid ? (int)(state.soc * 100.0f + 0.5f) : -1;
        widget_battery_draw(fb, 270,  86, soc_pct);
        widget_laptime_draw(fb,  18, 171, state.lap_count,
                            state.current_lap_ms, state.gps_fix_ok);
        widget_best_lap_draw(fb, 18, 199, state.best_lap_count,
                             state.best_lap_ms);
    }
    draw_lap_notice(millis());
    display_blit::show(fb, warn);
}

Task g_tasks[] = {
    { can_rx_update,   5, 0 },   // 200 Hz drain
    { gps_update,     20, 0 },   // 50 Hz UART drain
    { ntrip_update,   10, 0 },   // 100 Hz Wi-Fi/NTRIP RTCM forwarding
    { bms_update,    100, 0 },   // 10 Hz BLE BMS state machine
    { bms_can_tx_update, 100, 0 }, // 10 Hz BMS telemetry to logger/TMA-1
    { gnss_position_can_tx_update, 20, 0 }, // event-driven: send once per new RMC fix
    { gnss_status_can_tx_update, 200, 0 },  // 5 Hz GNSS/RTK status telemetry
    { lap_can_tx_update, 200, 0 },          // 5 Hz lap telemetry
    { hmi_update,     20, 0 },   // 50 Hz
    { display_update, 66, 0 },   // ~15 Hz
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    pinMode(PIN_TOUCH_CS, OUTPUT);
    digitalWrite(PIN_TOUCH_CS, HIGH);
    display_blit::begin();
    display_update();
    touch.begin();
    touch.setRotation(1);

    pinMode(PIN_PADDOCK, INPUT); // GPIO36 has no internal pull-up; PCB provides external 10k
    pinMode(PIN_TC, INPUT_PULLUP);
    pinMode(PIN_REGEN_BIT0, INPUT_PULLUP);
    pinMode(PIN_REGEN_BIT1, INPUT); // GPIO34 has no internal pull-up; PCB must provide external 10k
    pinMode(PIN_DEBUG, INPUT_PULLUP);
    pinMode(PIN_GPS_LAP_START, INPUT_PULLUP);
    can_bus::begin();
    gps_laptimer::begin();
    bms_ble::begin();
}

