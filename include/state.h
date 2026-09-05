// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
// [LOCKED] Cluster shared state bus. ONLY core/app_wiring.cpp may include this.
// Module files (src/modules/*) must never include state.h.

struct ClusterState {
    // received vehicle state (from CAN, sniffed off VCU feedback frames)
    // speed_rpm is the LCD/display aggregate: abs(left) until both sides are
    // seen, then avg(abs(left), abs(right)).
    float    speed_rpm    = 0.0f;
    float    speed_rpm_l  = 0.0f;
    float    bus_voltage  = 0.0f;   // V
    float    bus_current  = 0.0f;   // A
    float    soc          = 0.0f;   // 0..1
    bool     soc_valid    = false;  // false until BMS/VCU provides real SOC
    int      controller_temp = 0;   // C
    int      motor_temp      = 0;   // C
    uint8_t  controller_status = 0; // raw MCU->VCU status byte
    uint8_t  error1 = 0, error2 = 0, error3 = 0;   // raw error bitmaps
    bool     controller_l_seen = false;
    uint32_t controller_l_fb1_last_ms = 0;
    uint32_t controller_l_fb2_last_ms = 0;
    // Controller_R mirror of the above (dual motor — see CAN_PROTOCOL.md §7)
    float    speed_rpm_r    = 0.0f;
    // Display/lap-timer speed source. Temporary bench/vehicle test mode derives this
    // from motor-controller RPM until wheel speed sensors are available.
    float    vehicle_speed_kph = 0.0f;
    bool     vehicle_speed_valid = false;
    uint32_t vehicle_speed_last_rx_ms = 0;
    float    bus_voltage_r  = 0.0f;
    float    bus_current_r  = 0.0f;
    int      controller_temp_r = 0;
    int      motor_temp_r      = 0;
    uint8_t  controller_status_r = 0;
    uint8_t  error1_r = 0, error2_r = 0, error3_r = 0;
    bool     controller_r_seen = false;
    uint32_t controller_r_fb1_last_ms = 0;
    uint32_t controller_r_fb2_last_ms = 0;
    uint8_t  gear   = 0;            // display gear: 0=N, 1=R, 2=D, 3=P
    bool     gear_from_can = false; // true after VCU-confirmed status arrives
    uint32_t vcu_cluster_status_last_ms = 0;
    bool     brake     = false;
    bool     hv_active = false;
    float    throttle_pct = 0.0f;   // VCU-calibrated accelerator position, 0..100
    bool     throttle_valid = false;
    bool     handshaked = false;
    // HMI outputs (sent to VCU)
    bool     paddock = false;
    bool     tc_enabled = false;
    uint8_t  regen_level = 0;
    bool     debug_enabled = false;
    bool     reset_req  = false;


    // BMS telemetry (display-only). BMS data may arrive directly over BLE
    // or through a future VCU summary frame; it must not be used for safety
    // decisions in Cluster firmware.
    bool     bms_ble_connected = false;
    uint32_t bms_last_rx_ms    = 0;
    float    bms_pack_voltage  = 0.0f;  // V
    float    bms_current       = 0.0f;  // A, sign convention not yet verified
    int      bms_temp_c        = 0;
    uint32_t bms_remaining_mah = 0;
    uint8_t  bms_soh           = 0;
    uint16_t bms_cycles        = 0;
    // GPS lap timer
    bool     gps_fix_ok     = false;
    bool     gps_data_ok    = false;
    double   gps_latitude   = 0.0;
    double   gps_longitude  = 0.0;
    uint32_t gps_last_rx_ms = 0;  // last valid GPS/NMEA sentence reception
    uint32_t gps_pps_last_ms = 0; // last GNSS PPS rising edge
    uint32_t gps_pps_count   = 0; // PPS edge counter for diagnostics
    uint8_t  lap_count      = 0;
    uint32_t current_lap_ms = 0;
    uint32_t last_lap_ms    = 0;
    uint8_t  best_lap_count = 0;
    uint32_t best_lap_ms    = 0;
};

extern ClusterState state;   // defined in core/app_wiring.cpp

