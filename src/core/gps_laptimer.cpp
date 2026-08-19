#include "core/gps_laptimer.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "state.h"

namespace gps_laptimer {
namespace {
constexpr int PIN_GPS_RX = 35;      // GPS TX2 -> ESP32 GPIO35
constexpr int PIN_GPS_TX = 14;      // ESP32 GPIO14 -> GPS RX2 for RTCM3 corrections
constexpr int PIN_GNSS_PPS = 33;    // ZED-F9P PPS -> ESP32 GPIO33
constexpr uint32_t GPS_BAUD = 115200;
constexpr uint32_t GPS_RECOVERY_BAUD = 460800;
constexpr uint32_t GPS_BAUD_RETRY_MS = 5000;
constexpr uint32_t GPS_STARTUP_WAIT_MS = 1200;
constexpr uint8_t GPS_BAUD_RUNTIME_LAYERS = 0x01; // RAM
constexpr uint8_t GPS_BAUD_PERSIST_LAYERS = 0x07; // RAM + BBR + Flash
constexpr int GPS_LINE_MAX = 96;
constexpr int GGA_LINE_MAX = 96;
constexpr float FINISH_LINE_HALF_WIDTH_M = 5.0f;
constexpr float START_RADIUS_M = 2.0f;
constexpr float FINISH_HEADING_LOCK_DISTANCE_M = 5.0f;
constexpr float HEADING_SAMPLE_MIN_DISTANCE_M = 0.5f;
constexpr uint8_t HEADING_SAMPLE_WINDOW = 4;
constexpr uint8_t HEADING_SAMPLE_MIN_COUNT = 2;
constexpr float REARM_RADIUS_M = 20.0f;
constexpr float DEPART_SPEED_KPH = 1.5f;
constexpr uint32_t DEPART_CONFIRM_MS = 150;
constexpr uint32_t VEHICLE_SPEED_STALE_MS = 300;
constexpr uint32_t MIN_LAP_MS = 10000;
constexpr uint32_t GPS_FIX_TIMEOUT_MS = 3000;

HardwareSerial gps_serial(2);
char line[GPS_LINE_MAX];
int line_len = 0;
char last_gga[GGA_LINE_MAX];
uint32_t last_gga_time_ms = 0;
uint8_t gga_fix_quality = 0;
uint8_t gga_satellites = 0;
float gga_hdop = 0.0f;
uint32_t last_baud_force_ms = 0;
volatile uint32_t pps_last_ms_isr = 0;
volatile uint32_t pps_count_isr = 0;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR pps_isr() {
    pps_last_ms_isr = millis();
    ++pps_count_isr;
}

bool have_start = false;
bool lap_armed = false;
bool waiting_departure = false;
bool timing_active = false;
bool latest_fix = false;
double start_lat = 0.0;
double start_lon = 0.0;
double latest_lat = 0.0;
double latest_lon = 0.0;
bool finish_line_ready = false;
float finish_heading_x = 0.0f;
float finish_heading_y = 0.0f;
float heading_sample_x[HEADING_SAMPLE_WINDOW] = {};
float heading_sample_y[HEADING_SAMPLE_WINDOW] = {};
uint8_t heading_sample_count = 0;
uint8_t heading_sample_next = 0;
uint32_t last_cross_ms = 0;
uint32_t last_fix_ms = 0;
uint32_t departure_speed_since_ms = 0;

int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool checksum_ok(const char *s) {
    if (s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (!star) return true;

    uint8_t sum = 0;
    for (const char *p = s + 1; p < star; ++p) sum ^= (uint8_t)*p;

    int hi = hex_value(star[1]);
    int lo = hex_value(star[2]);
    if (hi < 0 || lo < 0) return false;
    return sum == (uint8_t)((hi << 4) | lo);
}

void send_ubx_baud(uint8_t port_key_byte, uint32_t baud, uint8_t layers) {
    uint8_t message[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00,
        0x00, layers, 0x00, 0x00,
        0x01, 0x00, port_key_byte, 0x40,
        (uint8_t)(baud & 0xFF),
        (uint8_t)((baud >> 8) & 0xFF),
        (uint8_t)((baud >> 16) & 0xFF),
        (uint8_t)((baud >> 24) & 0xFF),
        0x00, 0x00
    };

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < sizeof(message) - 2; ++i) {
        ck_a = (uint8_t)(ck_a + message[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    message[sizeof(message) - 2] = ck_a;
    message[sizeof(message) - 1] = ck_b;

    gps_serial.write(message, sizeof(message));
    gps_serial.flush();
}

void send_ubx_all_uart_baud(uint32_t baud, uint8_t layers) {
    // UBX-CFG-VALSET keys: UART1 baud 0x40520001, UART2 baud 0x40530001.
    send_ubx_baud(0x52, baud, layers);
    delay(20);
    send_ubx_baud(0x53, baud, layers);
}

void force_gps_baud_115200(bool persist, uint32_t startup_wait_ms) {
    const uint8_t layers = persist ? GPS_BAUD_PERSIST_LAYERS : GPS_BAUD_RUNTIME_LAYERS;

    gps_serial.end();
    gps_serial.begin(GPS_RECOVERY_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    if (startup_wait_ms > 0) delay(startup_wait_ms);

    // Try both known rates. If the receiver is already running at 115200 from
    // RAM while BBR/Flash still says 460800, the second write makes that
    // 115200 setting durable before the next power cycle.
    send_ubx_all_uart_baud(GPS_BAUD, layers);
    delay(100);
    gps_serial.end();
    line_len = 0;
    gps_serial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    send_ubx_all_uart_baud(GPS_BAUD, layers);
    last_baud_force_ms = millis();

    Serial.print("[GPS] UART1/UART2 forced to ");
    Serial.print(GPS_BAUD);
    Serial.println(persist ? " baud (RAM/BBR/Flash)" : " baud (RAM)");
}

float distance_m(double lat1, double lon1, double lat2, double lon2);
struct LocalPoint {
    float x;
    float y;
};

double deg_min_to_decimal(const char *value, char hemi) {
    if (!value || !value[0]) return 0.0;
    const double raw = atof(value);
    const int deg = (int)(raw / 100.0);
    const double minutes = raw - (double)deg * 100.0;
    double decimal = (double)deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') decimal = -decimal;
    return decimal;
}

void copy_gga_sentence(const char *sentence) {
    std::strncpy(last_gga, sentence, sizeof(last_gga) - 1);
    last_gga[sizeof(last_gga) - 1] = '\0';
    last_gga_time_ms = millis();
}

void parse_gga(char *sentence) {
    if (!checksum_ok(sentence)) return;

    char original[GGA_LINE_MAX];
    std::strncpy(original, sentence, sizeof(original) - 1);
    original[sizeof(original) - 1] = '\0';

    char *star = strchr(sentence, '*');
    if (star) *star = '\0';

    char *fields[16] = {};
    int count = 0;
    char *p = sentence[0] == '$' ? sentence + 1 : sentence;
    fields[count++] = p;
    while (*p && count < 16) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        ++p;
    }

    if (count < 7) return;
    if (strcmp(fields[0], "GPGGA") != 0 && strcmp(fields[0], "GNGGA") != 0) return;
    copy_gga_sentence(original);
    gga_fix_quality = (uint8_t)atoi(fields[6]);
    gga_satellites = count > 7 ? (uint8_t)atoi(fields[7]) : 0;
    gga_hdop = count > 8 ? (float)atof(fields[8]) : 0.0f;
}

bool vehicle_speed_fresh(uint32_t now) {
    return state.vehicle_speed_last_rx_ms != 0 &&
           (now - state.vehicle_speed_last_rx_ms) <= VEHICLE_SPEED_STALE_MS;
}

void update_departure_timer(uint32_t now) {
    if (!have_start || !waiting_departure) return;

    state.current_lap_ms = 0;
    if (!latest_fix) {
        departure_speed_since_ms = 0;
        return;
    }

    bool departed = false;
    if (vehicle_speed_fresh(now)) {
        departed = state.vehicle_speed_kph > DEPART_SPEED_KPH;
    } else {
        departed = distance_m(start_lat, start_lon, latest_lat, latest_lon) >= START_RADIUS_M;
    }

    if (!departed) {
        departure_speed_since_ms = 0;
        return;
    }

    if (departure_speed_since_ms == 0) {
        departure_speed_since_ms = now;
        return;
    }

    if (now - departure_speed_since_ms < DEPART_CONFIRM_MS) return;

    waiting_departure = false;
    timing_active = true;
    lap_armed = false;
    last_cross_ms = departure_speed_since_ms;
    departure_speed_since_ms = 0;
}

float distance_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    constexpr double DEG2RAD_LOCAL = 0.017453292519943295;
    const double p1 = lat1 * DEG2RAD_LOCAL;
    const double p2 = lat2 * DEG2RAD_LOCAL;
    const double dlat = (lat2 - lat1) * DEG2RAD_LOCAL;
    const double dlon = (lon2 - lon1) * DEG2RAD_LOCAL;
    const double x = dlon * cos((p1 + p2) * 0.5);
    return (float)(sqrt(x * x + dlat * dlat) * R);
}

LocalPoint to_start_local_m(double lat, double lon) {
    constexpr double R = 6371000.0;
    constexpr double DEG2RAD_LOCAL = 0.017453292519943295;
    const double p_start = start_lat * DEG2RAD_LOCAL;
    const double p = lat * DEG2RAD_LOCAL;
    const double dlat = (lat - start_lat) * DEG2RAD_LOCAL;
    const double dlon = (lon - start_lon) * DEG2RAD_LOCAL;
    const double x = dlon * cos((p_start + p) * 0.5) * R;
    const double y = dlat * R;
    return { (float)x, (float)y };
}

void reset_heading_samples() {
    for (uint8_t i = 0; i < HEADING_SAMPLE_WINDOW; ++i) {
        heading_sample_x[i] = 0.0f;
        heading_sample_y[i] = 0.0f;
    }
    heading_sample_count = 0;
    heading_sample_next = 0;
}

void add_heading_sample(double prev_lat, double prev_lon, double lat, double lon) {
    if (finish_line_ready) return;

    const LocalPoint p0 = to_start_local_m(prev_lat, prev_lon);
    const LocalPoint p1 = to_start_local_m(lat, lon);
    const float dx = p1.x - p0.x;
    const float dy = p1.y - p0.y;
    const float len = sqrtf(dx * dx + dy * dy);
    if (len < HEADING_SAMPLE_MIN_DISTANCE_M) return;

    heading_sample_x[heading_sample_next] = dx / len;
    heading_sample_y[heading_sample_next] = dy / len;
    heading_sample_next = (uint8_t)((heading_sample_next + 1) % HEADING_SAMPLE_WINDOW);
    if (heading_sample_count < HEADING_SAMPLE_WINDOW) ++heading_sample_count;
}

bool update_finish_line_heading(double lat, double lon) {
    if (finish_line_ready) return true;

    const LocalPoint p = to_start_local_m(lat, lon);
    const float d = sqrtf(p.x * p.x + p.y * p.y);
    if (d < FINISH_HEADING_LOCK_DISTANCE_M) return false;
    if (heading_sample_count < HEADING_SAMPLE_MIN_COUNT) return false;

    float avg_x = 0.0f;
    float avg_y = 0.0f;
    for (uint8_t i = 0; i < heading_sample_count; ++i) {
        avg_x += heading_sample_x[i];
        avg_y += heading_sample_y[i];
    }

    float avg_len = sqrtf(avg_x * avg_x + avg_y * avg_y);
    if (avg_len < 0.001f) return false;
    avg_x /= avg_len;
    avg_y /= avg_len;

    if (avg_x * p.x + avg_y * p.y < 0.0f) {
        avg_x = -avg_x;
        avg_y = -avg_y;
    }

    finish_heading_x = avg_x;
    finish_heading_y = avg_y;
    finish_line_ready = true;
    return true;
}

bool finish_line_crossed(double prev_lat, double prev_lon,
                         double lat, double lon,
                         uint32_t prev_ms, uint32_t now,
                         uint32_t &cross_ms) {
    if (!finish_line_ready || prev_ms == 0) return false;

    const LocalPoint p0 = to_start_local_m(prev_lat, prev_lon);
    const LocalPoint p1 = to_start_local_m(lat, lon);
    const float along0 = p0.x * finish_heading_x + p0.y * finish_heading_y;
    const float along1 = p1.x * finish_heading_x + p1.y * finish_heading_y;

    // Count only the same-direction crossing used when leaving the start line.
    if (!(along0 < 0.0f && along1 >= 0.0f)) return false;

    const float denom = along1 - along0;
    if (denom <= 0.0f) return false;

    const float t = -along0 / denom;
    if (t < 0.0f || t > 1.0f) return false;

    const float line_x = -finish_heading_y;
    const float line_y = finish_heading_x;
    const float lateral0 = p0.x * line_x + p0.y * line_y;
    const float lateral1 = p1.x * line_x + p1.y * line_y;
    const float lateral = lateral0 + (lateral1 - lateral0) * t;
    if (fabsf(lateral) > FINISH_LINE_HALF_WIDTH_M) return false;

    cross_ms = prev_ms + (uint32_t)((now - prev_ms) * t + 0.5f);
    return true;
}

void record_lap(uint32_t cross_ms) {
    const uint32_t lap_ms = cross_ms - last_cross_ms;
    const uint8_t completed_lap = state.lap_count < 99 ? (uint8_t)(state.lap_count + 1) : 99;
    state.last_lap_ms = lap_ms;
    if (state.best_lap_ms == 0 || lap_ms < state.best_lap_ms) {
        state.best_lap_ms = lap_ms;
        state.best_lap_count = completed_lap;
    }
    state.current_lap_ms = 0;
    last_cross_ms = cross_ms;
    if (state.lap_count < 99) ++state.lap_count;
    lap_armed = false;
}

void update_lap(double lat, double lon) {
    const uint32_t now = millis();
    const bool had_previous_fix = latest_fix;
    const double prev_lat = latest_lat;
    const double prev_lon = latest_lon;
    const uint32_t prev_fix_ms = last_fix_ms;

    state.gps_fix_ok = true;
    state.gps_data_ok = true;
    state.gps_latitude = lat;
    state.gps_longitude = lon;
    latest_fix = true;
    latest_lat = lat;
    latest_lon = lon;
    last_fix_ms = now;

    if (!have_start) {
        return;
    }

    if (waiting_departure) return;

    const float dist = distance_m(start_lat, start_lon, lat, lon);
    if (!timing_active || last_cross_ms == 0) return;

    state.current_lap_ms = now - last_cross_ms;

    if (had_previous_fix) {
        add_heading_sample(prev_lat, prev_lon, lat, lon);
    }
    update_finish_line_heading(lat, lon);

    if (dist >= REARM_RADIUS_M) {
        lap_armed = true;
    }

    uint32_t cross_ms = now;
    if (lap_armed && had_previous_fix &&
        finish_line_crossed(prev_lat, prev_lon, lat, lon, prev_fix_ms, now, cross_ms) &&
        cross_ms >= last_cross_ms &&
        cross_ms - last_cross_ms >= MIN_LAP_MS) {
        record_lap(cross_ms);
    }
}

void parse_rmc(char *sentence) {
    if (!checksum_ok(sentence)) return;

    char *star = strchr(sentence, '*');
    if (star) *star = '\0';

    char *fields[16] = {};
    int count = 0;
    char *p = sentence[0] == '$' ? sentence + 1 : sentence;
    fields[count++] = p;
    while (*p && count < 16) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        ++p;
    }

    if (count < 7) return;
    if (strcmp(fields[0], "GPRMC") != 0 && strcmp(fields[0], "GNRMC") != 0) return;

    if (fields[2][0] != 'A') {
        state.gps_fix_ok = false;
        latest_fix = false;
        last_fix_ms = 0;
        return;
    }

    const double lat = deg_min_to_decimal(fields[3], fields[4][0]);
    const double lon = deg_min_to_decimal(fields[5], fields[6][0]);
    update_lap(lat, lon);
}

void consume_char(char c) {
    if (c == '\r') return;
    if (c == '\n') {
        line[line_len] = '\0';
        if (line_len > 6) {
            if (line[0] == '$' && checksum_ok(line)) {
                state.gps_last_rx_ms = millis();
                state.gps_data_ok = true;
            }
            char gga_copy[GPS_LINE_MAX];
            std::strncpy(gga_copy, line, sizeof(gga_copy) - 1);
            gga_copy[sizeof(gga_copy) - 1] = '\0';
            parse_gga(gga_copy);
            parse_rmc(line);
        }
        line_len = 0;
        return;
    }

    if (line_len < GPS_LINE_MAX - 1) {
        line[line_len++] = c;
    } else {
        line_len = 0;
    }
}
}

void begin() {
    pinMode(PIN_GNSS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GNSS_PPS), pps_isr, RISING);
    force_gps_baud_115200(true, GPS_STARTUP_WAIT_MS);
    Serial.print("[GPS] UART2 RX GPIO");
    Serial.print(PIN_GPS_RX);
    Serial.print(" TX GPIO");
    Serial.print(PIN_GPS_TX);
    Serial.print(" baud ");
    Serial.println(GPS_BAUD);
    Serial.print("[GPS] PPS GPIO");
    Serial.println(PIN_GNSS_PPS);
}

void poll() {
    noInterrupts();
    const uint32_t pps_ms = pps_last_ms_isr;
    const uint32_t pps_count_snapshot = pps_count_isr;
    interrupts();
    state.gps_pps_last_ms = pps_ms;
    state.gps_pps_count = pps_count_snapshot;

    while (gps_serial.available() > 0) {
        consume_char((char)gps_serial.read());
    }

    const uint32_t now = millis();
    if ((latest_fix && now - last_fix_ms > GPS_FIX_TIMEOUT_MS) ||
        (state.gps_last_rx_ms != 0 && now - state.gps_last_rx_ms > GPS_FIX_TIMEOUT_MS)) {
        state.gps_fix_ok = false;
        latest_fix = false;
    }
    if (state.gps_last_rx_ms == 0 || now - state.gps_last_rx_ms > GPS_FIX_TIMEOUT_MS) {
        state.gps_data_ok = false;
    }

    const bool nmea_missing = state.gps_last_rx_ms == 0 ||
                              now - state.gps_last_rx_ms > GPS_FIX_TIMEOUT_MS;
    if (nmea_missing && now - last_baud_force_ms >= GPS_BAUD_RETRY_MS) {
        Serial.println("[GPS] NMEA timeout, retrying UART1/UART2 at 115200 baud");
        force_gps_baud_115200(false, 0);
    }

    update_departure_timer(now);
}

bool start_at_current_fix() {
    if (!latest_fix) return false;

    start_lat = latest_lat;
    start_lon = latest_lon;
    have_start = true;
    lap_armed = false;
    waiting_departure = true;
    timing_active = false;
    finish_line_ready = false;
    finish_heading_x = 0.0f;
    finish_heading_y = 0.0f;
    reset_heading_samples();
    last_cross_ms = 0;
    departure_speed_since_ms = 0;
    state.lap_count = 0;
    state.current_lap_ms = 0;
    state.last_lap_ms = 0;
    state.best_lap_count = 0;
    state.best_lap_ms = 0;
    return true;
}

void stop() {
    have_start = false;
    lap_armed = false;
    waiting_departure = false;
    timing_active = false;
    finish_line_ready = false;
    finish_heading_x = 0.0f;
    finish_heading_y = 0.0f;
    reset_heading_samples();
    last_cross_ms = 0;
    departure_speed_since_ms = 0;
    state.current_lap_ms = 0;
}

size_t write_rtcm(const uint8_t *data, size_t len) {
    if (!data || len == 0) return 0;
    return gps_serial.write(data, len);
}

const char *last_gga_sentence() {
    return last_gga;
}

uint32_t last_gga_ms() {
    return last_gga_time_ms;
}

uint32_t last_nmea_ms() {
    return state.gps_last_rx_ms;
}

uint32_t last_pps_ms() {
    return state.gps_pps_last_ms;
}

uint32_t pps_count() {
    return state.gps_pps_count;
}

uint8_t fix_quality() {
    return gga_fix_quality;
}

uint8_t satellites() {
    return gga_satellites;
}

float hdop() {
    return gga_hdop;
}

const char *rtk_status_label() {
    switch (gga_fix_quality) {
        case 4: return "RTK FIXED";
        case 5: return "RTK FLOAT";
        case 2: return "DGPS";
        case 1: return "GPS FIX";
        default: return "NO FIX";
    }
}
}

