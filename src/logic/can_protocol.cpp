// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "can_protocol.h"

EmVoltages decode_em_voltages(const uint8_t data[8]) {
    const auto signed_le = [](const uint8_t *p) -> int16_t {
        const uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
        return (int16_t)(raw >= 0x8000 ? (int32_t)raw - 65536 : (int32_t)raw);
    };
    return {signed_le(data), signed_le(data + 4)};
}

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
    const uint8_t regen_level = cmd.regen_level <= 3 ? cmd.regen_level : 0;
    const bool regen_enable = regen_level > 0;
    out[1] = (cmd.tc_enabled ? 0x01 : 0x00) |
             (regen_enable ? 0x02 : 0x00) |
             (cmd.debug_enabled ? 0x08 : 0x00);
    out[2] = (cmd.paddock ? 0x01 : 0x00);
    out[3] = 0xA0 | regen_level; // explicit stage; byte1 bit1 remains master ON
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

void encode_cluster_gnss_speed(const ClusterGnssSpeed &speed, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;
    uint16_t speed_raw = 0;
    if (speed.speed_valid && speed.speed_kph > 0.0f) {
        const float scaled = speed.speed_kph * 100.0f;
        speed_raw = scaled >= 65535.0f ? 0xFFFF : (uint16_t)(scaled + 0.5f);
    }
    put_u16le(out + 0, speed_raw);
    out[2] = (speed.rmc_fresh ? 0x01 : 0x00) |
             (speed.gps_fix_valid ? 0x02 : 0x00) |
             (speed.rtk_state == 1 ? 0x04 : 0x00) |
             (speed.rtk_state == 2 ? 0x08 : 0x00) |
             (speed.speed_valid ? 0x10 : 0x00);
    out[3] = speed.fix_quality;
    put_u16le(out + 4, speed.rmc_age_dsec);
    out[7] = speed.life;
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

void encode_cluster_lap_history(const ClusterLapHistoryFrame &lap, uint8_t out[8]) {
    out[0] = lap.lap_number;
    out[1] = (lap.valid ? 0x01u : 0u) |
             (lap.active ? 0x02u : 0u) |
             (lap.completed ? 0x04u : 0u) |
             (lap.timer_running ? 0x08u : 0u) |
             (lap.timer_paused ? 0x10u : 0u);
    put_u32le(out + 2, lap.lap_time_ms);
    out[6] = lap.session;
    out[7] = lap.life;
}

void encode_cluster_lap_battery(const ClusterLapHistoryFrame &lap, uint8_t out[8]) {
    out[0] = lap.lap_number;
    out[1] = (lap.battery_valid ? 0x01u : 0u) |
             (lap.active ? 0x02u : 0u) |
             (lap.completed ? 0x04u : 0u) |
             (lap.timer_running ? 0x08u : 0u) |
             (lap.timer_paused ? 0x10u : 0u);
    put_u16le(out + 2, lap.battery_valid ? lap.battery_used_x10 : 0u);
    out[4] = 0;
    out[5] = 0;
    out[6] = lap.session;
    out[7] = lap.life;
}

void encode_cluster_lap_temperature(const ClusterLapHistoryFrame &lap,
    const LapTemperatureSummary &temperature, bool rise, uint8_t out[8]) {
    out[0] = lap.lap_number;
    out[1] = (lap.valid && temperature.mean_valid[0] ? 0x01u : 0u) |
             (lap.valid && temperature.mean_valid[1] ? 0x02u : 0u) |
             (lap.valid && temperature.rise_valid[0] ? 0x04u : 0u) |
             (lap.valid && temperature.rise_valid[1] ? 0x08u : 0u) |
             (lap.completed ? 0x10u : 0u) | (lap.active ? 0x20u : 0u) |
             (lap.timer_running ? 0x40u : 0u) | (lap.timer_paused ? 0x80u : 0u);
    for (unsigned side = 0; side < 2; ++side) {
        const bool valid = lap.valid && (rise ? temperature.rise_valid[side] : temperature.mean_valid[side]);
        const int16_t value = valid ? (rise ? temperature.rise_x10[side] : temperature.mean_x10[side])
                                    : LAP_TEMPERATURE_INVALID;
        put_u16le(out + 2 + side * 2, static_cast<uint16_t>(value));
    }
    out[6] = lap.session;
    out[7] = lap.life;
}

void ClusterLapHistory::clear_session(bool emit_invalid) {
    if (emit_invalid && max_lap_seen_ != 0) {
        clear_lap_ = 1;
        clear_until_ = max_lap_seen_;
    } else {
        clear_lap_ = 0;
        clear_until_ = 0;
    }
    for (uint8_t i = 0; i < CLUSTER_LAP_HISTORY_MAX; ++i) {
        completed_ms_[i] = 0;
        completed_battery_x10_[i] = 0;
        completed_battery_valid_[i] = false;
    }
    latest_ = ClusterLapHistoryInput{};
    completed_count_ = 0;
    max_lap_seen_ = 0;
    completed_cursor_ = 0;
    resend_count_ = 0;
    resend_credit_ = 0;
    old_resend_credit_ = 0;
    recent_cursor_ = old_cursor_ = 0;
    final_lap_ = 0;
    final_repeats_ = 0;
    session_present_ = false;
}

void ClusterLapHistory::update(const ClusterLapHistoryInput &input) {
    if (input.current_lap_number == 0) {
        if (session_present_) {
            ++session_;
            session_reserved_ = true;
            clear_session(true);
        }
        latest_ = input;
        return;
    }

    if (!session_present_) {
        if (!session_reserved_) ++session_;
        session_reserved_ = false;
        session_present_ = true;
    } else if (input.completed_lap_count < completed_count_) {
        ++session_;
        clear_session(true);
        session_present_ = true;
    }

    const uint8_t completed = input.completed_lap_count > CLUSTER_LAP_HISTORY_MAX
        ? CLUSTER_LAP_HISTORY_MAX : input.completed_lap_count;
    if (completed > completed_count_) {
        // MIN_LAP_MS makes multiple crossings inside one 200 ms TX period
        // impossible, so last_lap_ms belongs to this newly completed slot.
        completed_ms_[completed - 1] = input.last_lap_ms;
        completed_battery_x10_[completed - 1] = input.last_battery_used_x10;
        completed_battery_valid_[completed - 1] = input.last_battery_valid;
        completed_count_ = completed;
        final_lap_ = completed;
        final_repeats_ = CLUSTER_LAP_HISTORY_FINAL_REPEATS;
    }
    latest_ = input;
    max_lap_seen_ = input.current_lap_number > max_lap_seen_
        ? input.current_lap_number : max_lap_seen_;
    if (completed_count_ > max_lap_seen_) max_lap_seen_ = completed_count_;
}

void ClusterLapHistory::fill_completed(uint8_t lap_number, ClusterLapHistoryFrame &frame) const {
    frame = ClusterLapHistoryFrame{};
    frame.lap_number = lap_number;
    frame.valid = true;
    frame.completed = true;
    frame.lap_time_ms = completed_ms_[lap_number - 1];
    frame.battery_used_x10 = completed_battery_x10_[lap_number - 1];
    frame.battery_valid = completed_battery_valid_[lap_number - 1];
    frame.session = session_;
}

bool ClusterLapHistory::take_transition_frame(ClusterLapHistoryFrame &frame) {
    if (clear_lap_ != 0 && clear_lap_ <= clear_until_) {
        frame = ClusterLapHistoryFrame{};
        frame.lap_number = clear_lap_++;
        frame.session = session_;
        if (clear_lap_ > clear_until_) clear_lap_ = clear_until_ = 0;
        return true;
    }
    if (final_repeats_ != 0 && final_lap_ != 0) {
        fill_completed(final_lap_, frame);
        --final_repeats_;
        return true;
    }
    return false;
}

bool ClusterLapHistory::active_frame(ClusterLapHistoryFrame &frame) const {
    if (!session_present_ || latest_.current_lap_number == 0 ||
        latest_.completed_lap_count >= CLUSTER_LAP_HISTORY_MAX) return false;
    frame = ClusterLapHistoryFrame{};
    frame.lap_number = latest_.current_lap_number;
    frame.valid = true;
    frame.active = true;
    frame.timer_running = latest_.timer_running;
    frame.timer_paused = latest_.timer_paused;
    frame.lap_time_ms = latest_.current_lap_ms;
    frame.battery_used_x10 = latest_.current_battery_used_x10;
    frame.battery_valid = latest_.current_battery_valid;
    frame.session = session_;
    return true;
}

bool ClusterLapHistory::next_completed_frame(ClusterLapHistoryFrame &frame) {
    if (completed_count_ == 0) return false;
    if (completed_cursor_ >= completed_count_) completed_cursor_ = 0;
    fill_completed(static_cast<uint8_t>(completed_cursor_ + 1), frame);
    ++completed_cursor_;
    return true;
}

bool ClusterLapHistory::due_completed_frame(uint32_t now_ms, ClusterLapHistoryFrame &frame) {
    if (resend_count_ != completed_count_) {
        resend_count_ = completed_count_;
        resend_ms_ = now_ms;
        resend_credit_ = 0;
        old_resend_credit_ = 0;
        recent_cursor_ = old_cursor_ = 0;
        return false;
    }
    const uint32_t elapsed = now_ms - resend_ms_;
    resend_ms_ = now_ms;
    if (completed_count_ == 0) return false;
    // Keep the active lap and nine recent completed laps fast. Older slots
    // retain their meaning but are spread over ten seconds, without catch-up.
    const uint32_t bounded_elapsed = elapsed > 1000 ? 1000 : elapsed;
    const uint8_t recent_count = completed_count_ < CLUSTER_LAP_RECENT_COMPLETED_MAX
        ? completed_count_ : CLUSTER_LAP_RECENT_COMPLETED_MAX;
    const uint8_t old_count = completed_count_ - recent_count;
    resend_credit_ += bounded_elapsed * recent_count;
    old_resend_credit_ += bounded_elapsed * old_count;
    // Service the slower pool first on a simultaneous deadline to avoid
    // starving old records; the recent pool remains due on the next call.
    if (old_count && old_resend_credit_ >= CLUSTER_LAP_OLD_RESEND_MS) {
        old_resend_credit_ %= CLUSTER_LAP_OLD_RESEND_MS;
        fill_completed(static_cast<uint8_t>(old_cursor_ + 1), frame);
        old_cursor_ = (old_cursor_ + 1) % old_count;
        return true;
    }
    if (resend_credit_ < CLUSTER_LAP_RECENT_RESEND_MS) return false;
    resend_credit_ %= CLUSTER_LAP_RECENT_RESEND_MS;
    fill_completed(static_cast<uint8_t>(old_count + recent_cursor_ + 1), frame);
    recent_cursor_ = (recent_cursor_ + 1) % recent_count;
    return true;
}

VcuClusterStatus decode_vcu_cluster_status(const uint8_t data[8]) {
    VcuClusterStatus out;
    out.gear_valid = data[0] <= 3;
    out.gear = out.gear_valid ? data[0] : 0;
    out.brake = (data[1] & 0x01) != 0;
    out.hv_active = (data[1] & 0x02) != 0;
    out.soc_valid = (data[1] & 0x04) != 0;
    out.throttle_valid = (data[1] & 0x08) != 0;
    out.paddock_active = (data[1] & 0x10) != 0;
    out.soc_pct = out.soc_valid ? (data[2] <= 100 ? data[2] : 100) : 0;
    out.throttle_pct = out.throttle_valid
        ? (data[3] <= 100 ? data[3] : 100) : 0;
    out.life = data[7];
    return out;
}

void decode_vcu_vehicle_speed(const uint8_t d[8], float &kph, bool &valid) {
    // 값은 플래그와 무관하게 해독한다. 표시에 쓸지는 받는 쪽이 정한다.
    valid = d[2] == 1;
    kph = (float)get_u16le(d) * 0.1f;
}

bool is_ezkontrol_handshake_probe(const uint8_t data[8]) {
    for (int i = 0; i < 8; ++i) {
        if (data[i] != 0x55) return false;
    }
    return true;
}

bool is_ezkontrol_handshake_ack(const uint8_t data[8]) {
    for (int i = 0; i < 8; ++i) {
        if (data[i] != 0xAA) return false;
    }
    return true;
}

float decode_motor_target_current_a(const uint8_t data[8]) {
    return raw_to_torque(get_u16le(data + 0));
}

void encode_reset_report(uint8_t reason, uint8_t rom_reason, uint32_t uptime_ms,
                         uint32_t resets_since_power_on, uint8_t life,
                         uint8_t out[8]) {
    out[0] = reason;
    out[1] = rom_reason;
    put_u16le(out + 2, (uint16_t)(uptime_ms & 0xFFFFu));
    put_u16le(out + 4, (uint16_t)(uptime_ms >> 16));
    out[6] = resets_since_power_on > 255u ? (uint8_t)255u
                                          : (uint8_t)resets_since_power_on;
    out[7] = life;
}
