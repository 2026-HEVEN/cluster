// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "cluster_command.h"
// [LOCKED] TWAI (ESP32 CAN). RX-first (vehicle state, stub) + Cluster->VCU command.

namespace can_bus {
    void begin();
    void poll_rx();                          // drain RX -> state (parse is a stub)
    void send_command(const ClusterCommand &cmd);   // encode -> 0x1801D0C0
    void send_bms_status();                  // encode -> 0x18F3FFC0 / 0x18F4FFC0
    void send_gnss_position();               // encode -> 0x18F5FFC0
    void send_gnss_rtk_status();             // encode -> 0x18F6FFC0
    void send_lap_time();                    // encode -> 0x18F7FFC0
    void send_lap_status(bool timer_running);// encode -> 0x18F8FFC0
}
