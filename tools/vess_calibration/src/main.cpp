#include <Arduino.h>

// Standalone VESS calibration sketch.
// This does not change the Cluster firmware.
//
// Wiring:
//   ESP32 GPIO26 -> VESS signal input
//   ESP32 GND    -> VESS GND/common ground
//
// Current Cluster firmware uses the same 50 Hz servo-style PWM output:
//   throttle -1.0 -> reverse pulse
//   throttle  0.0 -> idle/neutral pulse
//   throttle +1.0 -> full forward pulse

namespace {

constexpr int VESS_PWM_PIN = 26;
constexpr uint8_t VESS_PWM_CHANNEL = 0;
constexpr uint32_t VESS_PWM_FREQUENCY_HZ = 50;
constexpr uint8_t VESS_PWM_RESOLUTION_BITS = 16;

// Existing Cluster VESS pulse range.
// Reverse is provisional for calibration because the production reverse
// calibration value has not been confirmed on the real VESS unit yet.
constexpr uint16_t PULSE_REVERSE_US = 1000;
constexpr uint16_t PULSE_IDLE_US = 1500;
constexpr uint16_t PULSE_FORWARD_US = 2000;

// Human-facing calibration targets the team discussed.
// These are labels/levels for the VESS sound target, not ESP32 PWM pulse widths.
constexpr int SOUND_LEVEL_REVERSE = 15;
constexpr int SOUND_LEVEL_IDLE = 20;
constexpr int SOUND_LEVEL_FORWARD = 30;

float throttle = 0.0f;
uint16_t active_pulse_us = PULSE_IDLE_US;

uint32_t pulse_us_to_duty(uint16_t pulse_us) {
    const uint32_t period_us = 1000000UL / VESS_PWM_FREQUENCY_HZ;
    const uint32_t max_duty = (1UL << VESS_PWM_RESOLUTION_BITS) - 1UL;
    return ((uint32_t)pulse_us * max_duty + period_us / 2UL) / period_us;
}

float clamp_throttle(float value) {
    if (value < -1.0f) return -1.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

uint16_t pulse_for_throttle(float value) {
    value = clamp_throttle(value);
    if (value >= 0.0f) {
        return (uint16_t)(PULSE_IDLE_US +
                         (PULSE_FORWARD_US - PULSE_IDLE_US) * value + 0.5f);
    }
    return (uint16_t)(PULSE_IDLE_US +
                     (PULSE_IDLE_US - PULSE_REVERSE_US) * value - 0.5f);
}

int sound_level_for_throttle(float value) {
    value = clamp_throttle(value);
    if (value >= 0.0f) {
        return (int)(SOUND_LEVEL_IDLE +
                    (SOUND_LEVEL_FORWARD - SOUND_LEVEL_IDLE) * value + 0.5f);
    }
    return (int)(SOUND_LEVEL_IDLE +
                (SOUND_LEVEL_IDLE - SOUND_LEVEL_REVERSE) * value - 0.5f);
}

void write_pulse(uint16_t pulse_us) {
    active_pulse_us = pulse_us;
    ledcWrite(VESS_PWM_CHANNEL, pulse_us_to_duty(pulse_us));
}

void apply_throttle(float value) {
    throttle = clamp_throttle(value);
    write_pulse(pulse_for_throttle(throttle));
}

void print_status() {
    Serial.print("[VESS CAL] throttle=");
    Serial.print(throttle, 2);
    Serial.print(" pulse_us=");
    Serial.print(active_pulse_us);
    Serial.print(" sound_level=");
    Serial.println(sound_level_for_throttle(throttle));
}

void print_help() {
    Serial.println();
    Serial.println("===== VESS Calibration =====");
    Serial.println("GPIO26, 50 Hz servo-style PWM");
    Serial.println("i: idle      throttle  0.0, pulse 1500 us, level 20");
    Serial.println("f: full      throttle +1.0, pulse 2000 us, level 30");
    Serial.println("r: reverse   throttle -1.0, pulse 1000 us, level 15");
    Serial.println("0..9: forward throttle 0.0..0.9");
    Serial.println("+: increase throttle by 0.05");
    Serial.println("-: decrease throttle by 0.05");
    Serial.println("s: slow sweep -1.0 -> 0.0 -> +1.0 -> 0.0");
    Serial.println("p: print current status");
    Serial.println("h or ?: help");
    Serial.println("============================");
    print_status();
}

void run_sweep() {
    Serial.println("[VESS CAL] sweep start");
    for (int step = -20; step <= 20; ++step) {
        apply_throttle((float)step / 20.0f);
        print_status();
        delay(250);
    }
    for (int step = 20; step >= 0; --step) {
        apply_throttle((float)step / 20.0f);
        print_status();
        delay(250);
    }
    Serial.println("[VESS CAL] sweep end");
}

void handle_command(char c) {
    if (c >= '0' && c <= '9') {
        apply_throttle((float)(c - '0') / 10.0f);
    } else if (c == 'i' || c == 'I') {
        apply_throttle(0.0f);
    } else if (c == 'f' || c == 'F') {
        apply_throttle(1.0f);
    } else if (c == 'r' || c == 'R') {
        apply_throttle(-1.0f);
    } else if (c == '+') {
        apply_throttle(throttle + 0.05f);
    } else if (c == '-') {
        apply_throttle(throttle - 0.05f);
    } else if (c == 's' || c == 'S') {
        run_sweep();
    } else if (c == 'p' || c == 'P') {
        print_status();
        return;
    } else if (c == 'h' || c == 'H' || c == '?') {
        print_help();
        return;
    } else if (c == '\r' || c == '\n' || c == ' ') {
        return;
    } else {
        Serial.println("[VESS CAL] unknown command. Press h for help.");
        return;
    }
    print_status();
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(300);

    ledcSetup(VESS_PWM_CHANNEL, VESS_PWM_FREQUENCY_HZ, VESS_PWM_RESOLUTION_BITS);
    ledcAttachPin(VESS_PWM_PIN, VESS_PWM_CHANNEL);
    apply_throttle(0.0f);

    print_help();
}

void loop() {
    while (Serial.available() > 0) {
        handle_command((char)Serial.read());
    }
    delay(10);
}
