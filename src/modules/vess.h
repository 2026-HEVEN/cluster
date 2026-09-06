#pragma once
#include <cstdint>

struct VessInput {
    float vehicle_speed_kph;
    bool vehicle_speed_valid;
    float throttle_pct;
    bool throttle_valid;
    float torque_cmd_l_a;
    bool torque_cmd_l_valid;
    float torque_cmd_r_a;
    bool torque_cmd_r_valid;
    bool vehicle_on;
    uint8_t gear; // 0=N, 1=R, 2=D, 3=P
};

struct VessOutput {
    uint16_t pulse_us;
    int16_t throttle_percent;
};

VessOutput vess_compute(const VessInput &in);
