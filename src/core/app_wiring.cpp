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
#include "modules/vess.h"
#include "core/check_snapshot.h"
#include <algorithm>
#include <Arduino.h>
#include <XPT2046_Touchscreen.h>
#include <cstdio>

// [LOCKED] The ONLY translation unit that touches `state`.
ClusterState state;
static DiagnosticHistory diagnostic_history;
static TelemetryValues diagnostic_values;

namespace {
    // Input pins (direct GPIO; if pin count runs short, an io_expander can be
    // reintroduced HERE only, without touching any module).
    constexpr int PIN_PADDOCK = 36; // SVP/GPIO36; external 10k pull-up required on PCB
    constexpr int PIN_TC = 25;
    constexpr int PIN_REGEN_BIT0 = 27; // regen rotary bit 0; ON=LOW
    constexpr int PIN_REGEN_BIT1 = 34; // regen bit 1; external 10k pull-up to 3.3V
    constexpr int PIN_PAGE_BUTTON = 13; // WARNING_DETAIL momentary button; ON=LOW
    constexpr int PIN_VESS_PWM = 26;
    constexpr int PIN_GPS_LAP_START = 32;     // set GPS lap start
    constexpr int PIN_TOUCH_CS = 23;        // XPT2046 touch chip select; touch toggles vehicle status
    constexpr uint8_t VESS_PWM_CHANNEL = 0;
    constexpr uint32_t VESS_PWM_FREQUENCY_HZ = 50;
    constexpr uint8_t VESS_PWM_RESOLUTION_BITS = 16;
    constexpr uint32_t CAN_STARTUP_GRACE_MS = 3000;
    constexpr uint32_t CONTROLLER_FRAME_TIMEOUT_MS = 300;
    constexpr uint32_t VCU_STATUS_TIMEOUT_MS = 300;
    constexpr uint32_t CAN_WARNING_HOLD_MS = 1000;
    constexpr uint32_t VEHICLE_SPEED_TIMEOUT_MS = 300;
    constexpr uint32_t THROTTLE_TIMEOUT_MS = 300;
    constexpr uint32_t TORQUE_COMMAND_TIMEOUT_MS = 300;
    constexpr uint32_t GPS_SIGNAL_TIMEOUT_MS = 3000;
    constexpr uint32_t GPS_POSITION_CAN_STALE_MS = 3000;
    constexpr uint32_t LAP_NOTICE_MS = 1500;
    constexpr uint32_t NTRIP_START_DELAY_MS = 5000;
    constexpr float WHEEL_DIAMETER_M = 0.4597f;
    constexpr float MOTOR_TO_WHEEL_RATIO = 3.72f;
    constexpr float PI_F = 3.14159265f;
    // Raw calibration bounds need a four-corner check on the physical LCD.
    constexpr int TOUCH_MIN = 200, TOUCH_MAX = 3900;
    FrameBuffer fb;
    CheckUi check_ui;
    bool ui_warning = false;
    int touch_start_x = 0, touch_start_y = 0, touch_last_x = 0;
    bool touch_dragged = false;
    bool status_touch_down = false;
    bool page_button_raw_down = false;
    bool page_button_stable_down = false;
    uint32_t page_button_changed_ms = 0;
    uint32_t status_touch_last_ms = 0;
    constexpr uint32_t GPS_LAP_DOUBLE_CLICK_MS = 320;
    XPT2046_Touchscreen touch(PIN_TOUCH_CS);
    bool gps_lap_start_button_down = false;
    uint32_t gps_lap_start_button_last_ms = 0;
    bool gps_lap_start_single_pending = false;
    uint32_t gps_lap_start_click_ms = 0;
    const char *lap_notice_label = nullptr;
    uint32_t lap_notice_until_ms = 0;
    bool gps_fix_was_ok = false;
    bool ntrip_started = false;
    uint32_t last_gnss_position_seq_sent = 0;
    uint32_t can_warning_since_ms = 0;


    uint32_t vess_pulse_us_to_duty(uint16_t pulse_us) {
        const uint32_t period_us = 1000000UL / VESS_PWM_FREQUENCY_HZ;
        const uint32_t max_duty = (1UL << VESS_PWM_RESOLUTION_BITS) - 1UL;
        return ((uint32_t)pulse_us * max_duty + period_us / 2UL) / period_us;
    }

    void vess_write_pulse(uint16_t pulse_us) {
        ledcWrite(VESS_PWM_CHANNEL, vess_pulse_us_to_duty(pulse_us));
    }

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
        if (controller_fault_active()) {
            can_warning_since_ms = 0;
            return true;
        }

        if (!can_link_warning_active(now)) {
            can_warning_since_ms = 0;
            return false;
        }

        if (can_warning_since_ms == 0) {
            can_warning_since_ms = now;
            return false;
        }
        return now - can_warning_since_ms >= CAN_WARNING_HOLD_MS;
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

    void collect_warnings(CheckSnapshot &snapshot) {
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

        snapshot.warning_count = count;
        for (int i = 0; i < count; ++i) snapshot.warnings[i] = labels[i];
    }

    void page_button_update() {
        const uint32_t now = millis();
        const bool down = digitalRead(PIN_PAGE_BUTTON) == LOW;
        if (down != page_button_raw_down) {
            page_button_raw_down = down;
            page_button_changed_ms = now;
        }
        if (down != page_button_stable_down && now - page_button_changed_ms >= 50) {
            page_button_stable_down = down;
            if (down) check_ui.home();
        }
    }

    void status_touch_update() {
        const bool down = touch.touched();
        const uint32_t now = millis();
        if (down) {
            TS_Point p = touch.getPoint();
            const int x = std::max(0, std::min(319, (TOUCH_MAX-p.x)*319/(TOUCH_MAX-TOUCH_MIN)));
            const int y = std::max(0, std::min(239, (p.y-TOUCH_MIN)*239/(TOUCH_MAX-TOUCH_MIN)));
            if (!status_touch_down) {
                status_touch_down = true;
                touch_start_x = touch_last_x = x; touch_start_y = y;
                touch_dragged = false;
                status_touch_last_ms = now;
            } else {
                if (abs(x-touch_start_x)>10 || abs(y-touch_start_y)>10) touch_dragged = true;
                touch_last_x = x;
            }
        } else if (status_touch_down) {
            status_touch_down = false;
            if (!touch_dragged && now-status_touch_last_ms>=40) {
                check_ui.tap(touch_start_x,touch_start_y,ui_warning);
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
                if (gps_lap_start_single_pending &&
                    now - gps_lap_start_click_ms <= GPS_LAP_DOUBLE_CLICK_MS) {
                    gps_lap_start_single_pending = false;
                    gps_laptimer::reset();
                    show_lap_notice("GPS LAP RESET", now);
                } else {
                    gps_lap_start_single_pending = true;
                    gps_lap_start_click_ms = now;
                }
            }
        }

        if (gps_lap_start_single_pending &&
            now - gps_lap_start_click_ms > GPS_LAP_DOUBLE_CLICK_MS) {
            gps_lap_start_single_pending = false;
            if (gps_laptimer::timer_paused()) {
                if (gps_laptimer::resume()) show_lap_notice("LAP RESUMED", now);
            } else if (warning_active()) {
                if (gps_laptimer::stop()) show_lap_notice("LAP STOPPED", now);
            } else if (gps_laptimer::start_at_current_fix()) {
                show_lap_notice("LAP START SET", now);
            } else if (!gps_signal_fresh(now)) {
                show_lap_notice("NO GPS DATA", now);
            } else {
                show_lap_notice("NO GPS FIX", now);
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
        const bool vcu_speed_fresh =
            frame_fresh(state.vehicle_speed_last_rx_ms, now, VEHICLE_SPEED_TIMEOUT_MS);

        if (left_speed_fresh && right_speed_fresh) {
            state.speed_rpm = (absf(state.speed_rpm_l) + absf(state.speed_rpm_r)) * 0.5f;
        } else if (left_speed_fresh) {
            state.speed_rpm = absf(state.speed_rpm_l);
        } else if (right_speed_fresh) {
            state.speed_rpm = absf(state.speed_rpm_r);
        } else if (now >= CAN_STARTUP_GRACE_MS) {
            state.speed_rpm = 0.0f;
        }

        if (vcu_speed_fresh) {
            // VCU WSS frame is the preferred vehicle-speed source.
        } else if (left_speed_fresh || right_speed_fresh) {
            state.vehicle_speed_kph = motor_rpm_to_kph(state.speed_rpm);
            state.vehicle_speed_valid = true;
        } else {
            state.vehicle_speed_kph = 0.0f;
            state.vehicle_speed_valid = false;
        }

        if (state.gear_from_can && vcu_status_stale(now)) {
            state.gear_from_can = false;
            state.brake = false;
            state.hv_active = false;
            state.paddock_active = false;
            state.throttle_valid = false;
            state.throttle_last_rx_ms = 0;
        }
    }
}

static void hmi_update() {
    refresh_can_timeouts();
    gps_fix_feedback_update();
    gps_lap_start_update();
    page_button_update();
    status_touch_update();

    HmiSwitches sw;
    sw.paddock       = digitalRead(PIN_PADDOCK) == LOW;
    sw.tc_enabled    = digitalRead(PIN_TC) == LOW;
    sw.regen_bit0 = digitalRead(PIN_REGEN_BIT0) == LOW;
    sw.regen_bit1 = digitalRead(PIN_REGEN_BIT1) == LOW;
    sw.debug_enabled = false; // GPIO26 is now the local VESS PWM output.
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

static void vess_update() {
    const uint32_t now = millis();
    const bool throttle_fresh =
        frame_fresh(state.throttle_last_rx_ms, now, THROTTLE_TIMEOUT_MS) &&
        state.throttle_valid;
    const bool left_torque_fresh =
        frame_fresh(state.torque_cmd_l_last_ms, now, TORQUE_COMMAND_TIMEOUT_MS);
    const bool right_torque_fresh =
        frame_fresh(state.torque_cmd_r_last_ms, now, TORQUE_COMMAND_TIMEOUT_MS);
    const bool vehicle_on =
        throttle_fresh || left_torque_fresh || right_torque_fresh ||
        frame_fresh(state.vcu_cluster_status_last_ms, now, VCU_STATUS_TIMEOUT_MS) ||
        frame_fresh(state.vehicle_speed_last_rx_ms, now, VEHICLE_SPEED_TIMEOUT_MS) ||
        state.hv_active;
    const VessOutput out = vess_compute({
        state.vehicle_speed_kph,
        state.vehicle_speed_valid,
        state.throttle_pct,
        throttle_fresh,
        state.torque_cmd_l_a,
        left_torque_fresh,
        state.torque_cmd_r_a,
        right_torque_fresh,
        vehicle_on,
        state.gear,
    });
    vess_write_pulse(out.pulse_us);
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
static void diagnostics_update() { check_observe(diagnostic_history, diagnostic_values, millis()); }
static void display_update() {
    fb.clear();
    const bool warn = warning_active();
    ui_warning = warn;
    if (!warn && check_ui.page == CheckPage::Warning) check_ui.page = CheckPage::Menu;
    if (check_ui.page != CheckPage::Home) {
        static CheckSnapshot snapshot;
        if (check_ui.page != CheckPage::Graph) check_snapshot(snapshot, millis());
        if (check_ui.page == CheckPage::Warning) collect_warnings(snapshot);
        check_draw(fb, check_ui, snapshot, diagnostic_history, diagnostic_values, millis());
    } else {
        check_home_draw(fb, check_home_snapshot(millis()), warn);
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
    { vess_update,    20, 0 },   // 50 Hz VESS throttle-to-PWM output
    { diagnostics_update, 20, 0 }, // observe validity/edges; numeric history sampled at 2 Hz
    { display_update, 50, 0 },   // 20 Hz target; independent of history sampling
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    Serial.printf("[DIAGNOSTICS] history %u bytes, 2Hz/60s, volatile events\n",
                  static_cast<unsigned>(sizeof(diagnostic_history)));
    pinMode(PIN_TOUCH_CS, OUTPUT);
    digitalWrite(PIN_TOUCH_CS, HIGH);
    display_blit::begin();
    display_update();
    touch.begin();
    touch.setRotation(1);
    ledcSetup(VESS_PWM_CHANNEL, VESS_PWM_FREQUENCY_HZ, VESS_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_VESS_PWM, VESS_PWM_CHANNEL);
    vess_write_pulse(1500);

    pinMode(PIN_PADDOCK, INPUT); // GPIO36 has no internal pull-up; PCB provides external 10k
    pinMode(PIN_TC, INPUT_PULLUP);
    pinMode(PIN_REGEN_BIT0, INPUT_PULLUP);
    pinMode(PIN_REGEN_BIT1, INPUT); // external 10k pull-up required
    pinMode(PIN_PAGE_BUTTON, INPUT_PULLUP);
    pinMode(PIN_GPS_LAP_START, INPUT_PULLUP);
    can_bus::begin();
    gps_laptimer::begin();
    bms_ble::begin();
}

