// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "can_protocol.h"

uint16_t torque_to_raw(float amps) {
    return (uint16_t)((amps + 3200.0f) * 10.0f + 0.5f);
}

float raw_to_torque(uint16_t raw) {
    return (float)raw / 10.0f - 3200.0f;
}

float raw_to_voltage(uint16_t raw) { return (float)raw * 0.1f; }
float raw_to_current(uint16_t raw) { return (float)raw * 0.1f - 3200.0f; }
int   raw_to_temp(uint8_t raw)     { return (int)raw - 40; }
int   raw_to_speed(uint16_t raw)   { return (int)raw - 32000; }

namespace {
uint8_t clamp_u8(int value) {
    if (value < 0) return 0;
    if (value > 255) return 255;
    return (uint8_t)value;
}

uint8_t clamp_pct(int value) {
    if (value < 0) return 0;
    if (value > 100) return 100;
    return (uint8_t)value;
}

uint16_t clamp_u16(int32_t value) {
    if (value < 0) return 0;
    if (value > 65535) return 65535;
    return (uint16_t)value;
}

void put_u16le(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFF);
    out[1] = (uint8_t)(value >> 8);
}

uint16_t get_u16le(const uint8_t *d) {
    return (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
}

void put_u32le(uint8_t *out, uint32_t value) {
    out[0] = (uint8_t)(value & 0xFF);
    out[1] = (uint8_t)((value >> 8) & 0xFF);
    out[2] = (uint8_t)((value >> 16) & 0xFF);
    out[3] = (uint8_t)((value >> 24) & 0xFF);
}

void put_i32le(uint8_t *out, int32_t value) {
    put_u32le(out, (uint32_t)value);
}

int32_t clamp_i32_from_double(double value) {
    if (value < -2147483648.0) return (int32_t)0x80000000;
    if (value > 2147483647.0) return 2147483647;
    return (int32_t)(value >= 0.0 ? value + 0.5 : value - 0.5);
}
}

void encode_cluster_command(const ClusterCommand &cmd, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    const uint8_t regen_level = cmd.regen_level > 3 ? 3 : cmd.regen_level;
    out[1] = (cmd.tc_enabled ? 0x01 : 0x00) |
             (uint8_t)(regen_level << 1) |
             (cmd.debug_enabled ? 0x08 : 0x00) |
             (cmd.water_pump_auto_enabled ? 0x10 : 0x00);
    out[2] = (cmd.paddock ? 0x01 : 0x00);
}

void encode_cluster_bms_status(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    out[0] = (bms.valid ? 0x01 : 0x00) |
             (bms.ble_connected ? 0x02 : 0x00);
    out[1] = clamp_pct(bms.soc_pct);
    put_u16le(out + 2, clamp_u16((int32_t)(bms.pack_voltage_v * 10.0f + 0.5f)));
    put_u16le(out + 4, clamp_u16((int32_t)((bms.current_a + 3200.0f) * 10.0f + 0.5f)));
    out[6] = clamp_u8(bms.temp_c + 40);
    out[7] = life;
}

void encode_cluster_bms_detail(const ClusterBmsStatus &bms, uint8_t life, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    out[0] = (bms.valid ? 0x01 : 0x00) |
             (bms.ble_connected ? 0x02 : 0x00);
    out[1] = clamp_pct(bms.soh_pct);
    put_u16le(out + 2, clamp_u16((int32_t)bms.remaining_mah));
    put_u16le(out + 4, bms.cycles);
    out[7] = life;
}

void encode_cluster_gnss_position(const ClusterGnssPosition &pos, uint8_t out[8]) {
    const int32_t lat_raw = clamp_i32_from_double(pos.latitude_deg * 10000000.0);
    const int32_t lon_raw = clamp_i32_from_double(pos.longitude_deg * 10000000.0);
    put_i32le(out + 0, lat_raw);
    put_i32le(out + 4, lon_raw);
}

void encode_cluster_gnss_rtk_status(const ClusterGnssRtkStatus &status, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    out[0] = (status.gps_data_fresh ? 0x01 : 0x00) |
             (status.gps_fix_valid ? 0x02 : 0x00) |
             (status.ntrip_connected ? 0x04 : 0x00) |
             (status.rtcm_fresh ? 0x08 : 0x00);
    out[1] = status.fix_quality;
    out[2] = status.rtk_state > 2 ? 0 : status.rtk_state;
    out[3] = status.satellites;
    put_u16le(out + 4, clamp_u16((int32_t)(status.hdop * 10.0f + 0.5f)));
    put_u16le(out + 6, status.rtcm_age_dsec);
}

void encode_cluster_lap_time(uint32_t current_lap_ms, uint32_t last_lap_ms, uint8_t out[8]) {
    put_u32le(out + 0, current_lap_ms);
    put_u32le(out + 4, last_lap_ms);
}

void encode_cluster_lap_status(const ClusterLapStatus &lap, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    put_u32le(out + 0, lap.best_lap_ms);
    out[4] = lap.lap_count;
    out[5] = lap.best_lap_count;
    out[6] = lap.timer_running ? 0x01 : 0x00;
    out[7] = lap.life;
}

void decode_vcu_vehicle_speed(const uint8_t d[8], float &kph, bool &valid) {
    valid = d[2] == 1;
    kph = valid ? (float)get_u16le(d) * 0.1f : 0.0f;
}
