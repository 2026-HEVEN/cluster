#pragma once
#include "modules/diagnostics.h"
void check_snapshot(CheckSnapshot &snapshot, uint32_t now);
void check_observe(DiagnosticHistory &history, TelemetryValues &values, uint32_t now);
HomeData check_home_snapshot(uint32_t now);
