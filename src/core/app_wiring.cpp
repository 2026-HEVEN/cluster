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
#include "core/board_pins.h"
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
    // Physical GPIO assignments live in core/board_pins.h (PCB V3).
    constexpr uint8_t VESS_PWM_CHANNEL = 0;
    constexpr uint32_t VESS_MIN_FREQUENCY_HZ = 50;
    constexpr uint32_t VESS_MAX_FREQUENCY_HZ = 100;
    constexpr uint32_t VESS_EXTERNAL_HIGH_US = 2000;
    constexpr uint8_t VESS_PWM_RESOLUTION_BITS = 14;
    constexpr uint16_t START_INPUT_ON_MV = 1500;
    constexpr uint16_t START_INPUT_OFF_MV = 1000;
    constexpr uint32_t START_INPUT_DEBOUNCE_MS = 20;
    constexpr uint32_t REGEN_INPUT_DEBOUNCE_MS = 20;
    constexpr uint32_t CAN_STARTUP_GRACE_MS = 3000;
    constexpr uint32_t CONTROLLER_FRAME_TIMEOUT_MS = 300;
    constexpr uint32_t VCU_STATUS_TIMEOUT_MS = 300;
    constexpr uint32_t CAN_WARNING_HOLD_MS = 1000;
    constexpr uint32_t DRIVETRAIN_WARNING_HOLD_MS = 5000;
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
    XPT2046_Touchscreen touch(board_pins::TOUCH_CS);
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
    uint32_t drivetrain_warning_sequence_seen = 0;
    uint32_t vess_frequency_hz = 0;

    struct DebouncedInput {
        bool raw = false;
        bool stable = false;
        uint32_t changed_ms = 0;
    };
    DebouncedInput regen_bit0_input;
    DebouncedInput regen_bit1_input;
    bool start_input_candidate = false;
    uint32_t start_input_candidate_since_ms = 0;

    uint32_t vess_raw_high_duty(uint32_t frequency_hz) {
        const uint32_t period_counts = 1UL << VESS_PWM_RESOLUTION_BITS;
        const uint32_t max_duty = period_counts - 1UL;
        const uint32_t external_high_counts =
            ((uint64_t)period_counts * VESS_EXTERNAL_HIGH_US * frequency_hz + 500000UL) /
            1000000UL;
        const uint32_t raw_high_counts = period_counts - external_high_counts;
        return raw_high_counts > max_duty ? max_duty : raw_high_counts;
    }

    uint32_t vess_percent_to_frequency(int16_t percent) {
        int32_t magnitude = percent < 0 ? -(int32_t)percent : (int32_t)percent;
        if (magnitude > 100) magnitude = 100;
        return VESS_MIN_FREQUENCY_HZ +
               (uint32_t)magnitude * (VESS_MAX_FREQUENCY_HZ - VESS_MIN_FREQUENCY_HZ) / 100UL;
    }

    void vess_write_percent(int16_t percent) {
        const uint32_t frequency_hz = vess_percent_to_frequency(percent);
        if (frequency_hz != vess_frequency_hz) {
            ledcSetup(VESS_PWM_CHANNEL, frequency_hz, VESS_PWM_RESOLUTION_BITS);
            vess_frequency_hz = frequency_hz;
        }
        // PCB V3 uses one inverting NMOS stage. Keeping the MCU output HIGH
        // except for this 2 ms LOW interval creates a 2 ms external HIGH pulse.
        ledcWrite(VESS_PWM_CHANNEL, vess_raw_high_duty(frequency_hz));
    }

    bool debounce_active_low(DebouncedInput &input, int pin, uint32_t now) {
        const bool raw = digitalRead(pin) == LOW;
        if (raw != input.raw) {
            input.raw = raw;
            input.changed_ms = now;
        }
        if (input.stable != input.raw && now - input.changed_ms >= REGEN_INPUT_DEBOUNCE_MS) {
            input.stable = input.raw;
        }
        return input.stable;
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
        if (state.drivetrain_warning_sequence != 0 &&
            now - state.drivetrain_warning_started_ms < DRIVETRAIN_WARNING_HOLD_MS) {
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
        const bool drivetrain_visible = state.drivetrain_warning_sequence != 0 &&
            now - state.drivetrain_warning_started_ms < DRIVETRAIN_WARNING_HOLD_MS;
        if (drivetrain_visible) {
            add_warning(labels, count, state.drivetrain_last_fault == 2
                ? "DRIVE SYSTEM FAULT" : "DRIVE RPM WARNING");
            add_warning(labels, count, state.drivetrain_last_side == 0
                ? "LEFT MOTOR" : "RIGHT MOTOR");
        }

        snapshot.warning_count = count;
        for (int i = 0; i < count; ++i) snapshot.warnings[i] = labels[i];
    }

    void page_button_update() {
        const uint32_t now = millis();
        const bool down = digitalRead(board_pins::HOME_BUTTON) == LOW;
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
        const bool down = digitalRead(board_pins::LAP_BUTTON) == LOW;
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
            } else if (gps_laptimer::stop()) {
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

    const uint32_t now = millis();
    HmiSwitches sw;
    sw.paddock       = digitalRead(board_pins::PADDOCK_SWITCH) == LOW;
    sw.tc_enabled    = digitalRead(board_pins::TV_SWITCH) == LOW;
    sw.regen_bit0 = debounce_active_low(regen_bit0_input, board_pins::REGEN_BIT0, now);
    sw.regen_bit1 = debounce_active_low(regen_bit1_input, board_pins::REGEN_BIT1, now);
    sw.debug_enabled = false; // PCB V3 has no Debug switch input.
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
    vess_write_percent(out.throttle_percent);
}

static void start_input_update() {
    const uint32_t now = millis();
    const uint32_t measured_mv = analogReadMilliVolts(board_pins::START_SENSE_ADC);
    state.start_input_mv = measured_mv > UINT16_MAX ? UINT16_MAX : (uint16_t)measured_mv;
    state.start_input_valid = true;

    const bool candidate = state.start_input_present
        ? measured_mv > START_INPUT_OFF_MV
        : measured_mv >= START_INPUT_ON_MV;
    if (candidate != start_input_candidate) {
        start_input_candidate = candidate;
        start_input_candidate_since_ms = now;
    }
    if (candidate != state.start_input_present &&
        now - start_input_candidate_since_ms >= START_INPUT_DEBOUNCE_MS) {
        state.start_input_present = candidate;
    }
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
static void diagnostics_update() {
    const uint32_t now = millis();
    check_observe(diagnostic_history, diagnostic_values, now);
    if (state.drivetrain_warning_sequence != 0) {
        diagnostic_history.drivetrain(
            state.drivetrain_warning_sequence,
            state.drivetrain_last_side == 0 ? LinkId::MotorL : LinkId::MotorR,
            state.drivetrain_last_fault == 2
                ? EventKind::DriveDivergence : EventKind::DriveDropout,
            state.drivetrain_warning_started_ms);
    }
}
static void display_update() {
    fb.clear();
    if (state.drivetrain_warning_sequence != drivetrain_warning_sequence_seen) {
        drivetrain_warning_sequence_seen = state.drivetrain_warning_sequence;
        check_ui.page = CheckPage::Warning;
        check_ui.warning_page = 0;
    }
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
    { start_input_update, 5, 0 },           // 200 Hz PCB V3 START presence monitor
    { hmi_update,     20, 0 },   // 50 Hz
    { vess_update,    20, 0 },   // 50 Hz VESS throttle-to-PWM output
    { diagnostics_update, 20, 0 }, // observe validity/edges; numeric history sampled at 2 Hz
    { display_update, 50, 0 },   // 20 Hz target; independent of history sampling
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    Serial.printf("[DIAGNOSTICS] history %u bytes, 2Hz/60s, volatile events\n",
                  static_cast<unsigned>(sizeof(diagnostic_history)));
    pinMode(board_pins::TOUCH_CS, OUTPUT);
    digitalWrite(board_pins::TOUCH_CS, HIGH);
    display_blit::begin();
    display_update();
    touch.begin();
    touch.setRotation(1);
    pinMode(board_pins::VESS_PWM, OUTPUT);
    digitalWrite(board_pins::VESS_PWM, HIGH);
    ledcSetup(VESS_PWM_CHANNEL, VESS_MIN_FREQUENCY_HZ, VESS_PWM_RESOLUTION_BITS);
    ledcAttachPin(board_pins::VESS_PWM, VESS_PWM_CHANNEL);
    vess_write_percent(0);

    pinMode(board_pins::PADDOCK_SWITCH, INPUT);
    pinMode(board_pins::TV_SWITCH, INPUT);
    // GPIO36/39 are input-only and have no internal pull-up. PCB V3 supplies
    // external pull-ups for both active-low rotary bits.
    pinMode(board_pins::REGEN_BIT0, INPUT);
    pinMode(board_pins::REGEN_BIT1, INPUT);
    pinMode(board_pins::HOME_BUTTON, INPUT);
    pinMode(board_pins::LAP_BUTTON, INPUT);
    pinMode(board_pins::START_SENSE_ADC, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(board_pins::START_SENSE_ADC, ADC_11db);
    const uint32_t now = millis();
    regen_bit0_input.raw = regen_bit0_input.stable =
        digitalRead(board_pins::REGEN_BIT0) == LOW;
    regen_bit1_input.raw = regen_bit1_input.stable =
        digitalRead(board_pins::REGEN_BIT1) == LOW;
    regen_bit0_input.changed_ms = regen_bit1_input.changed_ms = now;
    start_input_candidate_since_ms = now;
    can_bus::begin();
    gps_laptimer::begin();
    bms_ble::begin();
}

