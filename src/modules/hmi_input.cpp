// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/hmi_input.h"

ClusterCommand hmi_compute(const HmiSwitches &in) {
    ClusterCommand cmd;
    cmd.paddock       = in.paddock;
    cmd.tc_enabled    = in.tc_enabled;
    cmd.regen_auto_enabled = in.regen_auto_enabled;
    cmd.debug_enabled = in.debug_enabled;

    return cmd;
}
