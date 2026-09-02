// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
#include "cluster_command.h"
// [SINGLE SOURCE OF TRUTH] Identical copy lives in the Cluster repo.
// Any edit here MUST be mirrored there. Owner: 김도현.

// --- CAN bus ---
constexpr uint32_t CAN_BITRATE = 250000;     // 250 kbps

// --- Node source addresses ---
constexpr uint8_t SA_VCU          = 0xD0;
constexpr uint8_t SA_CLUSTER      = 0xC0;
constexpr uint8_t SA_CONTROLLER_L = 0xEF;
constexpr uint8_t SA_CONTROLLER_R = 0xF0;
constexpr uint8_t SA_ENERGY_METER = 0x17;

// --- Torque command IDs (29-bit extended) ---
constexpr uint32_t CAN_ID_TORQUE_L = 0x0C01EFD0;
constexpr uint32_t CAN_ID_TORQUE_R = 0x0C01F0D0;

// --- Torque scaling: raw = (amps + 3200) * 10 ---
uint16_t torque_to_raw(float amps);
float    raw_to_torque(uint16_t raw);

// --- Cluster additions (mirror back into the VCU repo's can_protocol.h) ---
// MCU -> VCU feedback, sniffed passively off the shared bus (controllers stay
// in VCU mode; Cluster does not gateway/rebroadcast — see CAN_PROTOCOL.md §7).
constexpr uint32_t CAN_ID_FB1_L = 0x1801D0EF;   // Part I: voltage/current/speed (Controller_L)
constexpr uint32_t CAN_ID_FB2_L = 0x1802D0EF;   // Part II: temps/status/errors (Controller_L)
constexpr uint32_t CAN_ID_FB1_R = 0x1801D0F0;   // Part I (Controller_R)
constexpr uint32_t CAN_ID_FB2_R = 0x1802D0F0;   // Part II (Controller_R)
// Cluster -> VCU command (paddock/TC/regen enable/debug config). HEVEN-defined.
// The Cluster keeps a local regen level (0..3), but current VCU dev accepts
// only a boolean regen-auto request on the bus.
constexpr uint32_t CAN_ID_CLUSTER_CMD = 0x1801D0C0;
// VCU -> Cluster display status. HEVEN-defined. Used for VCU-confirmed gear,
// HV/brake state, and optional SOC when a battery interface is available.
constexpr uint32_t CAN_ID_VCU_CLUSTER_STATUS = 0x1801C0D0;
// VCU -> Cluster/TMA-1 single vehicle speed. Bytes 0..1 contain km/h x 10,
// byte 2 is valid flag (1=valid, 0=invalid), byte 3..7 reserved zero.
constexpr uint32_t CAN_ID_VCU_VEHICLE_SPEED = 0x1803C0D0;
// Cluster -> logger/TMA-1 BMS telemetry, broadcast. HEVEN-defined.
constexpr uint32_t CAN_ID_CLUSTER_BMS_STATUS = 0x18F3FFC0;
constexpr uint32_t CAN_ID_CLUSTER_BMS_DETAIL = 0x18F4FFC0;
// Cluster -> logger/TMA-1 GNSS/RTK/lap telemetry. HEVEN-defined.
constexpr uint32_t CAN_ID_CLUSTER_GNSS_POSITION = 0x18F5FFC0;
constexpr uint32_t CAN_ID_CLUSTER_GNSS_RTK_STATUS = 0x18F6FFC0;
constexpr uint32_t CAN_ID_CLUSTER_LAP_TIME = 0x18F7FFC0;
constexpr uint32_t CAN_ID_CLUSTER_LAP_STATUS = 0x18F8FFC0;

struct ClusterBmsStatus {
    bool     valid = false;
    bool     ble_connected = false;
    uint8_t  soc_pct = 0;
    float    pack_voltage_v = 0.0f;
    float    current_a = 0.0f;
    int      temp_c = 0;
    uint32_t remaining_mah = 0;
    uint8_t  soh_pct = 0;
    uint16_t cycles = 0;
};

struct ClusterGnssPosition {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};

struct ClusterGnssRtkStatus {
    bool gps_data_fresh = false;
    bool gps_fix_valid = false;
    bool ntrip_connected = false;
    bool rtcm_fresh = false;
    uint8_t fix_quality = 0;
    uint8_t rtk_state = 0; // 0=None, 1=Float, 2=Fixed
    uint8_t satellites = 0;
    float hdop = 0.0f;
    uint16_t rtcm_age_dsec = 0xFFFF;
};

struct ClusterLapStatus {
    uint32_t best_lap_ms = 0;
    uint8_t lap_count = 0;
    uint8_t best_lap_count = 0;
    bool timer_running = false;
    uint8_t life = 0;
};

// Cluster -> VCU command frame (0x1801D0C0) encoding. Mirror into VCU repo.
void encode_cluster_command(const ClusterCommand &cmd, uint8_t out[8]);
void encode_cluster_bms_status(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]);
void encode_cluster_bms_detail(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]);
void encode_cluster_gnss_position(const ClusterGnssPosition &pos, uint8_t out[8]);
void encode_cluster_gnss_rtk_status(const ClusterGnssRtkStatus &status, uint8_t out[8]);
void encode_cluster_lap_time(uint32_t current_lap_ms, uint32_t last_lap_ms, uint8_t out[8]);
void encode_cluster_lap_status(const ClusterLapStatus &lap, uint8_t out[8]);
void decode_vcu_vehicle_speed(const uint8_t d[8], float &kph, bool &valid);
bool is_ezkontrol_handshake_probe(const uint8_t data[8]);

// Signal decoders (EZkontrol scaling)
float raw_to_voltage(uint16_t raw);   // 0.1 V/bit, offset 0
float raw_to_current(uint16_t raw);   // 0.1 A/bit, offset -3200 A
int   raw_to_temp(uint8_t raw);       // 1 C/bit, offset -40 C
int   raw_to_speed(uint16_t raw);     // 1 rpm/bit, offset -32000 rpm (VCU path)
