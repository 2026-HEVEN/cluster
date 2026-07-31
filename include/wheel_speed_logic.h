#pragma once
#include <cstdint>

struct WheelSpeedConfig {
    float wheel_diameter_m;
};

struct FourWheelSpeed {
    uint16_t rpm_fl;
    uint16_t rpm_fr;
    uint16_t rpm_rl;
    uint16_t rpm_rr;
    float kph_fl;
    float kph_fr;
    float kph_rl;
    float kph_rr;
    float vehicle_kph;
};

FourWheelSpeed decode_wheel_speed_frame(const uint8_t data[8],
                                        const WheelSpeedConfig &config);
