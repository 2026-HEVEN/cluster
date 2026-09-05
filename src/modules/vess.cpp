#include "modules/vess.h"

namespace {
constexpr uint16_t PWM_REVERSE_US = 1000;
constexpr uint16_t PWM_NEUTRAL_US = 1500;
constexpr uint16_t PWM_FORWARD_US = 2000;

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
    if (!in.throttle_valid || (in.gear != 1 && in.gear != 2)) {
        return {PWM_NEUTRAL_US, 0};
    }

    float throttle_pct = in.throttle_pct;
    if (throttle_pct < 0.0f) throttle_pct = 0.0f;
    if (throttle_pct > 100.0f) throttle_pct = 100.0f;

    int16_t percent = (int16_t)(throttle_pct + 0.5f);
    if (in.gear == 1) percent = (int16_t)-percent;

    return {pulse_from_percent(percent), percent};
}

