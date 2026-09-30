#pragma once
#include <cstdint>

namespace ntrip {
// Runs Wi-Fi + NTRIP in its own task (core 0); getters are safe from any task.
void start_task();
bool wifi_connected();
bool connected();
const char *status_label();
uint32_t rtcm_bytes();
uint32_t last_rtcm_ms();
}
