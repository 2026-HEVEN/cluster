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
    constexpr int PIN_PADDOCK = 13;
    constexpr int PIN_TC = 14;
    constexpr int PIN_REGEN_ROTARY_BIT0 = 16; // 4-position rotary selector code bit0
    constexpr int PIN_REGEN_ROTARY_BIT1 = 17; // 4-position rotary selector code bit1
    constexpr int PIN_DEBUG = 27;
    constexpr int PIN_GPS_LAP_START = 19;     // set GPS lap start
    constexpr int PIN_TOUCH_CS = 21;        // XPT2046 touch chip select; touch toggles vehicle status
    constexpr int PIN_WARNING_DETAIL = 32; // LOW toggles warning detail page
    constexpr int PIN_LV_VOLTAGE = 34;   // ADC1, 100k/27k divider from LV 12V
    constexpr float LV_ADC_REF_V = 3.3f;
    constexpr float LV_ADC_MAX = 4095.0f;
    constexpr float LV_DIVIDER_SCALE = (100.0f + 27.0f) / 27.0f;
    constexpr uint32_t CAN_STARTUP_GRACE_MS = 3000;
    constexpr uint32_t CONTROLLER_FRAME_TIMEOUT_MS = 300;
    constexpr uint32_t VCU_STATUS_TIMEOUT_MS = 300;
    constexpr uint32_t WSS_FRAME_TIMEOUT_MS = 300;
    FrameBuffer fb;
    bool warning_detail_page = false;
    bool status_page = false;
    bool status_touch_down = false;
    uint32_t status_touch_last_ms = 0;
    constexpr uint32_t STATUS_TOUCH_DEBOUNCE_MS = 120;
    XPT2046_Touchscreen touch(PIN_TOUCH_CS);
    bool gps_lap_start_button_down = false;
    uint32_t gps_lap_start_button_last_ms = 0;
    bool warning_detail_button_down = false;
    uint32_t warning_detail_button_last_ms = 0;

    int gear_code(uint8_t gear) { return gear <= 3 ? gear : 0; }

    float absf(float v) { return v < 0.0f ? -v : v; }

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
        return state.vcu_cluster_status_last_ms != 0 &&
               frame_stale(state.vcu_cluster_status_last_ms, now, VCU_STATUS_TIMEOUT_MS);
    }

    bool can_link_warning_active(uint32_t now) {
        return controller_feedback_stale(now) || vcu_status_stale(now);
    }

    bool warning_active() {
        const uint32_t now = millis();
        return controller_fault_active() || can_link_warning_active(now);
    }

    bool bit_any(uint8_t left, uint8_t right, uint8_t bit) {
        return (left & (1u << bit)) || (right & (1u << bit));
    }

    void add_warning(const char *labels[], int &count, const char *label) {
        if (count < 8) labels[count++] = label;
    }

    void warning_line(int &y, const char *label, int scale, int step) {
        fb_text(fb, 18, y, label, scale);
        y += step;
    }

    bool status_page_active() {
        return status_page;
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

    void draw_side_status(int &y, const char *side, int motor_temp, int ctrl_temp,
                          float voltage, uint8_t err1, uint8_t err2, uint8_t err3) {
        char buf[48];

        std::snprintf(buf, sizeof(buf), "%s %s", side, fault_label(err1, err2, err3));
        status_line(y, buf, 3);

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

        fb_text(fb, 8, 4, "CAR CHECK", 3);

        int y = 31;
        std::snprintf(buf, sizeof(buf), "CAN L %s R %s",
                      dual_fresh_label(state.controller_l_fb1_last_ms,
                                       state.controller_l_fb2_last_ms,
                                       now, CONTROLLER_FRAME_TIMEOUT_MS),
                      dual_fresh_label(state.controller_r_fb1_last_ms,
                                       state.controller_r_fb2_last_ms,
                                       now, CONTROLLER_FRAME_TIMEOUT_MS));
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

        y += 4;
        draw_side_status(y, "LEFT", state.motor_temp, state.controller_temp,
                         state.bus_voltage, state.error1, state.error2, state.error3);

        y += 5;
        draw_side_status(y, "RIGHT", state.motor_temp_r, state.controller_temp_r,
                         state.bus_voltage_r, state.error1_r, state.error2_r, state.error3_r);
    }

    void draw_warning_detail() {
        const char *labels[8];
        int count = 0;
        const uint32_t now = millis();

        if (bit_any(state.error1, state.error1_r, 2)) add_warning(labels, count, "OVER VOLT");
        if (bit_any(state.error1, state.error1_r, 3)) add_warning(labels, count, "LOW VOLT");
        if (bit_any(state.error1, state.error1_r, 4)) add_warning(labels, count, "CTRL HOT");
        if (bit_any(state.error1, state.error1_r, 5)) add_warning(labels, count, "MOTOR HOT");
        if (state.error1 || state.error2 || state.error3) add_warning(labels, count, "LEFT FAULT");
        if (state.error1_r || state.error2_r || state.error3_r) add_warning(labels, count, "RIGHT FAULT");
        if (bit_any(state.error3, state.error3_r, 4) || can_link_warning_active(now)) {
            add_warning(labels, count, "CAN ERR");
        }
        if (count == 0) add_warning(labels, count, "FAULT");

        fb_text(fb, 18, 14, "WARNING", 5);

        int scale = 5;
        int step = 48;
        int y = count == 1 ? 112 : 78;
        if (count >= 4) {
            scale = 4;
            step = 39;
            y = 76;
        }
        if (count >= 6) {
            scale = 3;
            step = 30;
            y = 74;
        }

        for (int i = 0; i < count; ++i) warning_line(y, labels[i], scale, step);
    }

    uint8_t read_regen_rotary_level() {
        const uint8_t bit0 = digitalRead(PIN_REGEN_ROTARY_BIT0) == LOW ? 0x01 : 0x00;
        const uint8_t bit1 = digitalRead(PIN_REGEN_ROTARY_BIT1) == LOW ? 0x02 : 0x00;
        return bit0 | bit1;
    }

    void status_touch_update() {
        const bool down = touch.touched();
        const uint32_t now = millis();
        if (down != status_touch_down && now - status_touch_last_ms >= STATUS_TOUCH_DEBOUNCE_MS) {
            status_touch_down = down;
            status_touch_last_ms = now;
            if (down) {
                status_page = !status_page;
            }
        }
    }
    void gps_lap_start_update() {
        if (warning_active()) {
            gps_laptimer::stop();
            return;
        }

        warning_detail_page = false;
        const bool down = digitalRead(PIN_GPS_LAP_START) == LOW;
        const uint32_t now = millis();
        if (down != gps_lap_start_button_down && now - gps_lap_start_button_last_ms >= 50) {
            gps_lap_start_button_down = down;
            gps_lap_start_button_last_ms = now;
            if (down) {
                gps_laptimer::start_at_current_fix();
            }
        }
    }

    void warning_detail_update() {
        const bool warn = warning_active();
        if (!warn) {
            warning_detail_page = false;
            return;
        }

        const bool down = digitalRead(PIN_WARNING_DETAIL) == LOW;
        const uint32_t now = millis();
        if (down != warning_detail_button_down && now - warning_detail_button_last_ms >= 50) {
            warning_detail_button_down = down;
            warning_detail_button_last_ms = now;
            if (down) {
                warning_detail_page = !warning_detail_page;
            }
        }
    }

    void refresh_can_timeouts() {
        const uint32_t now = millis();
        const bool left_speed_fresh =
            frame_fresh(state.controller_l_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS);
        const bool right_speed_fresh =
            frame_fresh(state.controller_r_fb1_last_ms, now, CONTROLLER_FRAME_TIMEOUT_MS);

        if (now >= CAN_STARTUP_GRACE_MS) {
            if (left_speed_fresh && right_speed_fresh) {
                state.speed_rpm = (absf(state.speed_rpm_l) + absf(state.speed_rpm_r)) * 0.5f;
            } else if (left_speed_fresh) {
                state.speed_rpm = absf(state.speed_rpm_l);
            } else if (right_speed_fresh) {
                state.speed_rpm = absf(state.speed_rpm_r);
            } else {
                state.speed_rpm = 0.0f;
            }
        }

        if (frame_stale(state.wheel_speeds_last_rx_ms, now, WSS_FRAME_TIMEOUT_MS)) {
            state.vehicle_speed_kph = 0.0f;
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
    gps_lap_start_update();
    warning_detail_update();
    status_touch_update();

    HmiSwitches sw;
    sw.paddock       = digitalRead(PIN_PADDOCK) == LOW;
    sw.tc_enabled    = digitalRead(PIN_TC) == LOW;
    sw.regen_level   = read_regen_rotary_level();
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
static void bms_update() { bms_ble::poll(); }
static void bms_can_tx_update() { can_bus::send_bms_status(); }
static void lv_voltage_update() {
    const int raw = analogRead(PIN_LV_VOLTAGE);
    state.lv_voltage = ((float)raw * LV_ADC_REF_V / LV_ADC_MAX) * LV_DIVIDER_SCALE;
    state.lv_voltage_valid = true;
}

static void display_update() {
    fb.clear();
    const bool warn = warning_active();
    if (status_page_active()) {
        draw_vehicle_status();
    } else if (warn && warning_detail_page) {
        draw_warning_detail();
    } else {
        widget_speed_draw(fb,    10,  10, (int)(state.vehicle_speed_kph + 0.5f));
        widget_warnings_draw(fb, 248,  22, warn, state.hv_active);
        widget_gear_draw(fb,     289,  16, gear_code(state.gear));
        const int soc_pct = state.soc_valid ? (int)(state.soc * 100.0f + 0.5f) : -1;
        widget_battery_draw(fb, 285,  48, soc_pct);
        widget_laptime_draw(fb,  10, 136, state.lap_count,
                            state.current_lap_ms, state.gps_fix_ok);
        widget_best_lap_draw(fb, 205, 207, state.best_lap_count,
                             state.best_lap_ms);
    }
    display_blit::show(fb, warn);
}

Task g_tasks[] = {
    { can_rx_update,   5, 0 },   // 200 Hz drain
    { gps_update,     20, 0 },   // 50 Hz UART drain
    { bms_update,    100, 0 },   // 10 Hz BLE BMS state machine
    { bms_can_tx_update, 100, 0 }, // 10 Hz BMS telemetry to logger/TMA-1
    { lv_voltage_update, 100, 0 }, // 10 Hz LV 12V monitor
    { hmi_update,     20, 0 },   // 50 Hz
    { display_update, 66, 0 },   // ~15 Hz
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    pinMode(PIN_PADDOCK, INPUT_PULLUP);
    pinMode(PIN_TC, INPUT_PULLUP);
    pinMode(PIN_REGEN_ROTARY_BIT0, INPUT_PULLUP);
    pinMode(PIN_REGEN_ROTARY_BIT1, INPUT_PULLUP);
    pinMode(PIN_DEBUG, INPUT_PULLUP);
    pinMode(PIN_GPS_LAP_START, INPUT_PULLUP);
    pinMode(PIN_WARNING_DETAIL, INPUT_PULLUP);
    analogSetPinAttenuation(PIN_LV_VOLTAGE, ADC_11db);
    can_bus::begin();
    gps_laptimer::begin();
    bms_ble::begin();
    pinMode(PIN_TOUCH_CS, OUTPUT);
    digitalWrite(PIN_TOUCH_CS, HIGH);
    display_blit::begin();
    touch.begin();
}
