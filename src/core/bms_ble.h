#pragma once

namespace bms_ble {
// Runs scan/connect/poll in its own task (core 0); results land in `state`.
void start_task();
}