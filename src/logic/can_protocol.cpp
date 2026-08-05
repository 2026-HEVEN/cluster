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
}

void encode_cluster_command(const ClusterCommand &cmd, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    out[1] = (cmd.tc_enabled ? 0x01 : 0x00) |
             (cmd.regen_auto_enabled ? 0x02 : 0x00) |
             (cmd.debug_enabled ? 0x08 : 0x00);
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
