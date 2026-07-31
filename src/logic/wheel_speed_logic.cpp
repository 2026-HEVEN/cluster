#include "wheel_speed_logic.h"

namespace {
constexpr float PI_F = 3.14159265f;

uint16_t u16le(const uint8_t *data) {
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
}

float rpm_to_kph(uint16_t rpm, float wheel_diameter_m) {
    return (float)rpm * PI_F * wheel_diameter_m * 0.06f;
}

float trimmed_mean_four(float a, float b, float c, float d) {
    const float sum = a + b + c + d;
    float min_value = a;
    float max_value = a;
    if (b < min_value) min_value = b;
    if (c < min_value) min_value = c;
    if (d < min_value) min_value = d;
    if (b > max_value) max_value = b;
    if (c > max_value) max_value = c;
    if (d > max_value) max_value = d;
    return (sum - min_value - max_value) * 0.5f;
}
}

FourWheelSpeed decode_wheel_speed_frame(const uint8_t data[8],
                                        const WheelSpeedConfig &config) {
    FourWheelSpeed result{};
    result.rpm_fl = u16le(data + 0);
    result.rpm_fr = u16le(data + 2);
    result.rpm_rl = u16le(data + 4);
    result.rpm_rr = u16le(data + 6);

    result.kph_fl = rpm_to_kph(result.rpm_fl, config.wheel_diameter_m);
    result.kph_fr = rpm_to_kph(result.rpm_fr, config.wheel_diameter_m);
    result.kph_rl = rpm_to_kph(result.rpm_rl, config.wheel_diameter_m);
    result.kph_rr = rpm_to_kph(result.rpm_rr, config.wheel_diameter_m);
    result.vehicle_kph = trimmed_mean_four(
        result.kph_fl, result.kph_fr, result.kph_rl, result.kph_rr);
    return result;
}