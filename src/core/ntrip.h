#pragma once
#include <cstdint>

namespace ntrip {
void begin();
void poll();
bool wifi_connected();
bool connected();
uint32_t rtcm_bytes();
uint32_t last_rtcm_ms();
}
