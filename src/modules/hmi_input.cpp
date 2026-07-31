// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/hmi_input.h"

ClusterCommand hmi_compute(const HmiSwitches &in) {
    ClusterCommand cmd;
    cmd.paddock       = in.paddock;
    cmd.tc_enabled    = in.tc_enabled;
    cmd.regen_level   = in.regen_level & 0x03;
    cmd.debug_enabled = in.debug_enabled;

    return cmd;
}
