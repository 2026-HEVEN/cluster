// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
#include "modules/lap_temperature.h"
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
constexpr uint8_t SA_EM_GW = 0xC1;
constexpr uint32_t CAN_ID_EM_RECORD = 0x1CF5FFC1;
constexpr uint32_t CAN_ID_EM_SYNC = 0x1CF6FFC1;

struct EmVoltages {
    int16_t hv_decivolts;
    int16_t lv_centivolts;
};
EmVoltages decode_em_voltages(const uint8_t data[8]);

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
// byte1 bit1: regen master enable; byte3=0xA0..0xA3: explicit stage 0..3.
// Always sends rotary position; interpretation is configured by updated VCU.
constexpr uint32_t CAN_ID_CLUSTER_CMD = 0x1801D0C0;
// VCU -> Cluster display status. HEVEN-defined. Carries VCU-confirmed gear,
// HV/brake state, optional SOC, and calibrated throttle percent for VESS.
// byte1 bit3=throttle valid, bit4=Paddock active,
// byte3=throttle percent (0..100).
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
constexpr uint32_t CAN_ID_CLUSTER_GNSS_SPEED = 0x18F9FFC0;
// Cluster -> logger loop timing diagnostics, 1 Hz. HEVEN-defined.
// Every field covers the 1 s window since the previous frame.
// b0 slowest scheduler task index (g_tasks order)  b1-2 its longest run, ms
// b3-4 longest main-loop stall, ms (gap between 200 Hz CAN drains)
// b5 longest 0x1801D0C0 send interval, ms (saturating)
// b6 command sends later than 40 ms (saturating)  b7 life
constexpr uint32_t CAN_ID_CLUSTER_LOOP_TIMING = 0x18FAFFC0;
// Cluster -> logger multiplexed per-lap history. Byte 0 is the lap-number
// multiplexer so Monolith can expose Lap 1, Lap 2, ... as stable signals.
constexpr uint32_t CAN_ID_CLUSTER_LAP_HISTORY = 0x18FBFFC0;
// Same lap-number multiplexer as LAP_HISTORY, carrying BMS SOC consumed by
// that lap. Separate ID keeps the existing lap-time decoder contract stable.
constexpr uint32_t CAN_ID_CLUSTER_LAP_BATTERY = 0x18FCFFC0;
constexpr uint32_t CAN_ID_CLUSTER_LAP_CONTROLLER_MEAN = 0x18FDFFC0;
constexpr uint32_t CAN_ID_CLUSTER_LAP_CONTROLLER_RISE = 0x18FEFFC0;

// Node reset report, same layout on every ESP32 node (0x1CFDFF00 | SA), 1 s.
// Repeated so a logger that rebooted at the same moment still records it.
// b0 esp_reset_reason  b1 ROM reset reason (CPU0)  b2-5 uptime ms LE
// b6 resets since power-on (saturating)  b7 life
constexpr uint32_t CAN_ID_RESET_REPORT_BASE    = 0x1CFDFF00;
constexpr uint32_t CAN_ID_CLUSTER_RESET_REPORT = CAN_ID_RESET_REPORT_BASE | SA_CLUSTER;

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

struct ClusterGnssSpeed {
    float speed_kph = 0.0f;
    bool rmc_fresh = false;
    bool gps_fix_valid = false;
    bool speed_valid = false;
    uint8_t rtk_state = 0; // 0=None, 1=Float, 2=Fixed
    uint8_t fix_quality = 0;
    uint16_t rmc_age_dsec = 0xFFFF;
    uint8_t life = 0;
};

struct ClusterLapStatus {
    uint32_t best_lap_ms = 0;
    uint8_t lap_count = 0;
    uint8_t best_lap_count = 0;
    bool timer_running = false;
    uint8_t life = 0;
};

constexpr uint8_t CLUSTER_LAP_HISTORY_MAX = 99;
constexpr uint8_t CLUSTER_LAP_HISTORY_FINAL_REPEATS = 3;

struct ClusterLapHistoryInput {
    uint8_t current_lap_number = 0;  // 0=no configured Start/Finish line
    uint8_t completed_lap_count = 0;
    uint32_t current_lap_ms = 0;
    uint32_t last_lap_ms = 0;
    uint16_t current_battery_used_x10 = 0;
    uint16_t last_battery_used_x10 = 0;
    bool current_battery_valid = false;
    bool last_battery_valid = false;
    bool timer_running = false;
    bool timer_paused = false;
};

struct ClusterLapHistoryFrame {
    uint8_t lap_number = 0;
    bool valid = false;
    bool active = false;
    bool completed = false;
    bool timer_running = false;
    bool timer_paused = false;
    uint32_t lap_time_ms = 0;
    uint16_t battery_used_x10 = 0;
    bool battery_valid = false;
    uint8_t session = 0;
    uint8_t life = 0;
};

// Keeps the logger-facing lap slots stable without changing GPS crossing,
// pause/resume, or best-lap logic. One instance is owned by CAN TX.
class ClusterLapHistory {
public:
    void update(const ClusterLapHistoryInput &input);
    bool take_transition_frame(ClusterLapHistoryFrame &frame);
    bool active_frame(ClusterLapHistoryFrame &frame) const;
    bool next_completed_frame(ClusterLapHistoryFrame &frame);
    bool due_completed_frame(uint32_t now_ms, ClusterLapHistoryFrame &frame);
    uint8_t session() const { return session_; }

private:
    uint32_t completed_ms_[CLUSTER_LAP_HISTORY_MAX]{};
    uint16_t completed_battery_x10_[CLUSTER_LAP_HISTORY_MAX]{};
    bool completed_battery_valid_[CLUSTER_LAP_HISTORY_MAX]{};
    ClusterLapHistoryInput latest_{};
    uint8_t completed_count_ = 0;
    uint8_t max_lap_seen_ = 0;
    uint8_t completed_cursor_ = 0;
    uint8_t resend_count_ = 0;
    uint32_t resend_ms_ = 0;
    uint32_t resend_credit_ = 0;
    uint8_t final_lap_ = 0;
    uint8_t final_repeats_ = 0;
    uint8_t clear_lap_ = 0;
    uint8_t clear_until_ = 0;
    uint8_t session_ = 0;
    bool session_present_ = false;
    bool session_reserved_ = false;

    void clear_session(bool emit_invalid);
    void fill_completed(uint8_t lap_number, ClusterLapHistoryFrame &frame) const;
};

struct VcuClusterStatus {
    uint8_t gear = 0;
    bool gear_valid = false;
    bool brake = false;
    bool hv_active = false;
    bool soc_valid = false;
    uint8_t soc_pct = 0;
    bool throttle_valid = false;
    uint8_t throttle_pct = 0;
    bool paddock_active = false;
    uint8_t life = 0;
};

// Cluster -> VCU command frame (0x1801D0C0) encoding. Mirror into VCU repo.
void encode_cluster_command(const ClusterCommand &cmd, uint8_t out[8]);
void encode_cluster_bms_status(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]);
void encode_cluster_bms_detail(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]);
void encode_cluster_gnss_position(const ClusterGnssPosition &pos, uint8_t out[8]);
void encode_cluster_gnss_rtk_status(const ClusterGnssRtkStatus &status, uint8_t out[8]);
void encode_cluster_gnss_speed(const ClusterGnssSpeed &speed, uint8_t out[8]);
void encode_cluster_lap_time(uint32_t current_lap_ms, uint32_t last_lap_ms, uint8_t out[8]);
void encode_cluster_lap_status(const ClusterLapStatus &lap, uint8_t out[8]);
void encode_cluster_lap_history(const ClusterLapHistoryFrame &lap, uint8_t out[8]);
void encode_cluster_lap_battery(const ClusterLapHistoryFrame &lap, uint8_t out[8]);
void encode_cluster_lap_temperature(const ClusterLapHistoryFrame &lap,
    const LapTemperatureSummary &temperature, bool rise, uint8_t out[8]);
void encode_reset_report(uint8_t reason, uint8_t rom_reason, uint32_t uptime_ms,
                         uint32_t resets_since_power_on, uint8_t life,
                         uint8_t out[8]);
VcuClusterStatus decode_vcu_cluster_status(const uint8_t data[8]);
void decode_vcu_vehicle_speed(const uint8_t d[8], float &kph, bool &valid);
bool is_ezkontrol_handshake_probe(const uint8_t data[8]);
bool is_ezkontrol_handshake_ack(const uint8_t data[8]);
float decode_motor_target_current_a(const uint8_t data[8]);

// Signal decoders (EZkontrol scaling)
float raw_to_voltage(uint16_t raw);   // 0.1 V/bit, offset 0
float raw_to_current(uint16_t raw);   // 0.1 A/bit, offset -3200 A
int   raw_to_temp(uint8_t raw);       // 1 C/bit, offset -40 C
int   raw_to_speed(uint16_t raw);     // 1 rpm/bit, offset -32000 rpm (VCU path)
