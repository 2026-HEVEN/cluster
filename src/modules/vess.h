#pragma once
#include <cstdint>

struct VessInput {
    float throttle_pct;
    bool throttle_valid;
    uint8_t gear; // 0=N, 1=R, 2=D, 3=P
};

struct VessOutput {
    uint16_t pulse_us;
    int16_t throttle_percent;
};

VessOutput vess_compute(const VessInput &in);

