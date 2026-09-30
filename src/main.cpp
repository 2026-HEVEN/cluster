// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include <Arduino.h>
#include "core/wiring.h"
#include "scheduler_logic.h"
#include "core/board_pins.h"

// [LOCKED] Entry point. No life-signal task (Cluster is not torque-critical).
void setup() {
    // Hold CAN TXD recessive (HIGH) from the first instruction until TWAI
    // takes the pin; only the ROM boot window before this stays uncovered.
    pinMode(board_pins::CAN_TX, OUTPUT);
    digitalWrite(board_pins::CAN_TX, HIGH);
    Serial.begin(115200);
    modules_init();
}

void loop() {
    scheduler_run();
}
