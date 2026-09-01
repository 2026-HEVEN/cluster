// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/hmi_input.h"

ClusterCommand hmi_compute(const HmiSwitches &in) {
    ClusterCommand cmd;
    cmd.paddock       = in.paddock;
    cmd.tc_enabled    = in.tc_enabled;
    cmd.regen_level   = (in.regen_bit0 ? 1u : 0u) |
                        (in.regen_bit1 ? 2u : 0u);
    cmd.debug_enabled = in.debug_enabled;
    cmd.water_pump_auto_enabled = in.water_pump_auto_enabled;

    return cmd;
}
