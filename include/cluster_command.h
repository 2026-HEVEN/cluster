// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
// [LOCKED] Contract between the HMI module and the CAN layer: the "meaning"
// the Cluster sends to the VCU. The CAN byte encoding lives in can_protocol.

struct ClusterCommand {
    bool    paddock       = false; // request VCU speed limit
    bool    tc_enabled    = false; // traction control / torque vectoring request
    uint8_t regen_level   = 0;     // 0..3 locally; VCU command encodes >0 as regen on
    bool    debug_enabled = false; // request verbose/debug logging
};
