#include "modules/vess.h"

namespace {
constexpr uint16_t PWM_REVERSE_US = 1000;
constexpr uint16_t PWM_NEUTRAL_US = 1500;
constexpr uint16_t PWM_FORWARD_US = 2000;
constexpr float MIN_ACTIVE_SPEED_KPH = 0.5f;
constexpr float FULL_SCALE_SPEED_KPH = 20.0f;
constexpr float FULL_SCALE_TORQUE_A = 40.0f;
constexpr int16_t IDLE_SOUND_PERCENT = 18;

uint16_t pulse_from_percent(int16_t percent) {
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    const int32_t pulse = (int32_t)PWM_NEUTRAL_US + (int32_t)percent * 5L;
    if (pulse < PWM_REVERSE_US) return PWM_REVERSE_US;
    if (pulse > PWM_FORWARD_US) return PWM_FORWARD_US;
    return (uint16_t)pulse;
}

float absf(float value) {
    return value < 0.0f ? -value : value;
}

int16_t percent_from_torque(const VessInput &in) {
    if (!in.torque_cmd_l_valid && !in.torque_cmd_r_valid) return 0;

    float torque_a = 0.0f;
    if (in.torque_cmd_l_valid && in.torque_cmd_r_valid) {
        torque_a = (absf(in.torque_cmd_l_a) + absf(in.torque_cmd_r_a)) * 0.5f;
    } else if (in.torque_cmd_l_valid) {
        torque_a = absf(in.torque_cmd_l_a);
    } else {
        torque_a = absf(in.torque_cmd_r_a);
    }

    if (torque_a > FULL_SCALE_TORQUE_A) torque_a = FULL_SCALE_TORQUE_A;
    int16_t percent = (int16_t)((torque_a * 100.0f / FULL_SCALE_TORQUE_A) + 0.5f);
    if (percent > 0 && percent < IDLE_SOUND_PERCENT) percent = IDLE_SOUND_PERCENT;
    if (in.gear == 1) percent = (int16_t)-percent;
    return percent;
}

int16_t percent_from_throttle(const VessInput &in) {
    if (!in.throttle_valid) return 0;
    float pct = in.throttle_pct;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    int16_t percent = IDLE_SOUND_PERCENT +
        (int16_t)((pct * (100.0f - (float)IDLE_SOUND_PERCENT) / 100.0f) + 0.5f);
    if (in.gear == 1) percent = (int16_t)-percent;
    return percent;
}
}

VessOutput vess_compute(const VessInput &in) {
    const int16_t throttle_percent = percent_from_throttle(in);
    if (throttle_percent != 0) {
        return {pulse_from_percent(throttle_percent), throttle_percent};
    }

    const int16_t torque_percent = percent_from_torque(in);
    if (torque_percent != 0) {
        return {pulse_from_percent(torque_percent), torque_percent};
    }

    if (!in.vehicle_speed_valid || in.vehicle_speed_kph < MIN_ACTIVE_SPEED_KPH) {
        if (in.vehicle_on) {
            return {pulse_from_percent(IDLE_SOUND_PERCENT), IDLE_SOUND_PERCENT};
        }
        return {PWM_NEUTRAL_US, 0};
    }

    float speed_kph = in.vehicle_speed_kph;
    if (speed_kph < 0.0f) speed_kph = -speed_kph;
    if (speed_kph > FULL_SCALE_SPEED_KPH) speed_kph = FULL_SCALE_SPEED_KPH;

    int16_t percent = (int16_t)((speed_kph * 100.0f / FULL_SCALE_SPEED_KPH) + 0.5f);
    if (percent > 0 && percent < IDLE_SOUND_PERCENT) percent = IDLE_SOUND_PERCENT;
    if (in.gear == 1) percent = (int16_t)-percent;

    return {pulse_from_percent(percent), percent};
}
