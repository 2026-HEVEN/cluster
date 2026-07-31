#pragma once
#include <cstdint>
#include "cluster_command.h"
// [FILL-IN] Pure: physical switch states -> a semantic ClusterCommand.
// The locked core reads the switches and encodes the command to CAN.

struct HmiSwitches {
    bool     paddock;       // paddock-mode switch
    bool     tc_enabled;    // traction control / torque vectoring switch
    uint8_t  regen_level;   // 0..3 from the 4-position regen rotary selector
    bool     debug_enabled; // debug/logging switch
};

ClusterCommand hmi_compute(const HmiSwitches &in);
