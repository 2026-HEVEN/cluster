#pragma once
#include <cstdint>
#include "cluster_command.h"
// [FILL-IN] Pure: physical switch states -> a semantic ClusterCommand.
// The locked core reads the switches and encodes the command to CAN.

struct HmiSwitches {
    bool     paddock;       // paddock-mode switch
    bool     tc_enabled;    // traction control / torque vectoring switch
    bool     regen_auto_enabled; // regen auto toggle; false=request regen off
    bool     debug_enabled; // debug/logging switch
};

ClusterCommand hmi_compute(const HmiSwitches &in);
