#pragma once
#include <cstdint>
#include "cluster_command.h"
// [FILL-IN] Pure: physical switch states -> a semantic ClusterCommand.
// The locked core reads the switches and encodes the command to CAN.

struct HmiSwitches {
    bool     paddock;       // paddock-mode switch
    bool     tc_enabled;    // traction control / torque vectoring switch
    bool     regen_bit0;    // active-low physical regen rotary bit 0
    bool     regen_bit1;    // active-low physical regen rotary bit 1
    bool     debug_enabled; // debug/logging switch
    bool     water_pump_auto_enabled; // water pump auto enable switch
};

ClusterCommand hmi_compute(const HmiSwitches &in);
