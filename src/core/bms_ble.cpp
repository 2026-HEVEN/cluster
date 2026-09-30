#include "core/bms_ble.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "bms_protocol.h"
#include "state.h"

namespace bms_ble {
namespace {
constexpr char BMS_NAME[] = "LWS-1608";
constexpr uint32_t SCAN_RETRY_MS = 15000;
constexpr uint32_t SUMMARY_POLL_MS = 1000;
constexpr uint32_t STALE_MS = 5000;
constexpr int SCAN_SECONDS = 1;
// The 1 s scan, connect() and the 160 ms handshake all block. They run in
// their own task so the display/CAN scheduler never stalls on them (bench
// 2026-09-30: with the BMS link down the cluster went silent on CAN for
// 1.1 s every 15 s). Notify callbacks already arrive on the NimBLE host task.
constexpr uint32_t TASK_PERIOD_MS = 100;
constexpr uint32_t TASK_STACK = 6144;
constexpr UBaseType_t TASK_PRIORITY = 1;
constexpr BaseType_t TASK_CORE = 0; // with the BLE host, away from loop()

NimBLEUUID SVC("FFE0");
NimBLEUUID CH_WR("FFE1");
NimBLEUUID CH_NFY("FFE2");

NimBLEClient *client = nullptr;
NimBLERemoteCharacteristic *write_ch = nullptr;
uint32_t last_scan_ms = 0;
uint32_t last_summary_poll_ms = 0;

uint8_t frame_buf[64];
int frame_len_used = 0;

// bms_last_rx_ms is written from the NimBLE notify callback and may be a
// few ms newer than `now`; unsigned `now - stamp` would wrap.
bool summary_fresh(uint32_t now) {
    const uint32_t stamp = state.bms_last_rx_ms;
    return stamp != 0 && (int32_t)(now - stamp) <= (int32_t)STALE_MS;
}

void mark_disconnected() {
    write_ch = nullptr;
    frame_len_used = 0;
    state.bms_ble_connected = false;
    if (!summary_fresh(millis())) {
        state.soc_valid = false;
    }
}

void apply_summary(const BmsSummary &summary) {
    state.bms_pack_voltage = summary.pack_voltage_v;
    state.bms_current = summary.current_a;
    state.bms_temp_c = summary.temp_c;
    state.bms_remaining_mah = summary.remaining_mah;
    state.soc = (float)summary.soc_pct * 0.01f;
    state.soc_valid = true;
    state.bms_soh = summary.soh_pct;
    state.bms_cycles = summary.cycles;
    state.bms_last_rx_ms = millis();
    state.bms_ble_connected = true;
}

void handle_frame(const uint8_t *f, int len) {
    BmsSummary summary;
    if (bms_decode_summary_frame(f, len, summary)) apply_summary(summary);
}

void feed_byte(uint8_t b) {
    if (frame_len_used == 0 && b != 0x3A) return;
    if (frame_len_used == 1 && b != 0x16) {
        frame_len_used = 0;
        return;
    }

    if (frame_len_used >= (int)sizeof(frame_buf)) {
        frame_len_used = 0;
        return;
    }
    frame_buf[frame_len_used++] = b;

    if (frame_len_used < 3) return;
    const int want = bms_expected_frame_len(frame_buf[2]);
    if (want == 0) {
        frame_len_used = 0;
        return;
    }
    if (frame_len_used < want) return;

    if (frame_buf[want - 2] == 0x0D && frame_buf[want - 1] == 0x0A) {
        handle_frame(frame_buf, want);
    }
    frame_len_used = 0;
}

void on_notify(NimBLERemoteCharacteristic*, uint8_t *data, size_t len, bool) {
    for (size_t i = 0; i < len; ++i) feed_byte(data[i]);
}

void write_raw(const uint8_t *data, size_t len) {
    if (write_ch) write_ch->writeValue((uint8_t*)data, len, false);
}

void send_handshake() {
    const uint8_t h1[] = { 0x00, 0xA1, 0xA2, 0xA3 };
    write_raw(h1, sizeof(h1));
    delay(80);

    const uint8_t h2[] = {
        0xAA, 0x55, 0xF3, 0x01, 0x77, 0x77, 0x7C, 0x3F, 0x5A, 0xF3
    };
    write_raw(h2, sizeof(h2));
    delay(80);
}

void poll_frame(uint8_t id) {
    const uint8_t request_sum = (uint8_t)((0x16 + id) & 0xFF);
    const uint8_t frame[] = { 0x3A, 0x16, id, 0x00, request_sum, 0x00, 0x0D, 0x0A };
    write_raw(frame, sizeof(frame));
}

bool matches_bms(NimBLEAdvertisedDevice &dev) {
    return dev.haveName() && dev.getName() == BMS_NAME;
}

bool connect_device(NimBLEAdvertisedDevice &dev) {
    if (!client) client = NimBLEDevice::createClient();
    if (!client) return false;

    if (client->isConnected()) client->disconnect();
    if (!client->connect(&dev)) {
        mark_disconnected();
        return false;
    }

    NimBLERemoteService *svc = client->getService(SVC);
    if (!svc) {
        client->disconnect();
        mark_disconnected();
        return false;
    }

    NimBLERemoteCharacteristic *notify_ch = svc->getCharacteristic(CH_NFY);
    write_ch = svc->getCharacteristic(CH_WR);
    if (!notify_ch || !write_ch || !notify_ch->canNotify()) {
        client->disconnect();
        mark_disconnected();
        return false;
    }

    if (!notify_ch->subscribe(true, on_notify)) {
        client->disconnect();
        mark_disconnected();
        return false;
    }

    state.bms_ble_connected = false;
    frame_len_used = 0;
    send_handshake();
    poll_frame(0x2A);
    last_summary_poll_ms = millis();
    return true;
}

bool scan_and_connect() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (!scan) return false;

    scan->setActiveScan(true);
    NimBLEScanResults results = scan->start(SCAN_SECONDS, false);
    for (int i = 0; i < results.getCount(); ++i) {
        NimBLEAdvertisedDevice dev = results.getDevice(i);
        if (matches_bms(dev)) {
            const bool ok = connect_device(dev);
            scan->clearResults();
            return ok;
        }
    }

    scan->clearResults();
    mark_disconnected();
    return false;
}

bool connected() {
    return client && client->isConnected() && write_ch;
}

void poll() {
    const uint32_t now = millis();
    if (!connected()) {
        mark_disconnected();
        if (now - last_scan_ms >= SCAN_RETRY_MS) {
            last_scan_ms = now;
            scan_and_connect();
        }
        return;
    }

    const bool fresh = summary_fresh(now);
    state.bms_ble_connected = fresh;
    if (!fresh) {
        state.soc_valid = false;
    }

    if (now - last_summary_poll_ms >= SUMMARY_POLL_MS) {
        last_summary_poll_ms = now;
        poll_frame(0x2A);
    }
}

void task(void *) {
    for (;;) {
        poll();
        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
} // namespace

void start_task() {
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    last_scan_ms = millis() - SCAN_RETRY_MS;
    xTaskCreatePinnedToCore(task, "bms_ble", TASK_STACK, nullptr, TASK_PRIORITY,
                            nullptr, TASK_CORE);
}
}
