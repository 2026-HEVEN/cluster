#pragma once
#include <cstddef>
#include <cstdint>

namespace gps_laptimer {
void begin();
void poll();
bool start_at_current_fix();
void stop();
size_t write_rtcm(const uint8_t *data, size_t len);
const char *last_gga_sentence();
uint32_t last_gga_ms();
uint32_t last_nmea_ms();
uint32_t last_pps_ms();
uint32_t pps_count();
uint32_t position_sequence();
bool timer_running();
float gga_rate_hz();
float rmc_rate_hz();
uint8_t fix_quality();
uint8_t satellites();
float hdop();
const char *rtk_status_label();
}

