#include "modules/vess.h"

namespace {
constexpr uint16_t PWM_REVERSE_US = 1000;
constexpr uint16_t PWM_NEUTRAL_US = 1500;
constexpr uint16_t PWM_FORWARD_US = 2000;
constexpr float MIN_ACTIVE_SPEED_KPH = 0.5f;
constexpr float FULL_SCALE_SPEED_KPH = 40.0f;

uint16_t pulse_from_percent(int16_t percent) {
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    const int32_t pulse = (int32_t)PWM_NEUTRAL_US + (int32_t)percent * 5L;
    if (pulse < PWM_REVERSE_US) return PWM_REVERSE_US;
    if (pulse > PWM_FORWARD_US) return PWM_FORWARD_US;
    return (uint16_t)pulse;
}
}

VessOutput vess_compute(const VessInput &in) {
    if (!in.vehicle_speed_valid || in.vehicle_speed_kph < MIN_ACTIVE_SPEED_KPH) {
        return {PWM_NEUTRAL_US, 0};
    }

    float speed_kph = in.vehicle_speed_kph;
    if (speed_kph < 0.0f) speed_kph = -speed_kph;
    if (speed_kph > FULL_SCALE_SPEED_KPH) speed_kph = FULL_SCALE_SPEED_KPH;

    int16_t percent = (int16_t)((speed_kph * 100.0f / FULL_SCALE_SPEED_KPH) + 0.5f);
    if (in.gear == 1) percent = (int16_t)-percent;

    return {pulse_from_percent(percent), percent};
}

