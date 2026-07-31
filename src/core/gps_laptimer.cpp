#include "core/gps_laptimer.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "state.h"

namespace gps_laptimer {
namespace {
constexpr int PIN_GPS_RX = 22;      // GPS TX -> ESP32 GPIO22
constexpr int PIN_GPS_TX = -1;      // receive-only
constexpr uint32_t GPS_BAUD = 38400;
constexpr int GPS_LINE_MAX = 96;
constexpr float START_RADIUS_M = 2.0f;
constexpr float REARM_RADIUS_M = 20.0f;
constexpr uint32_t MIN_LAP_MS = 10000;
constexpr uint32_t GPS_FIX_TIMEOUT_MS = 3000;

HardwareSerial gps_serial(2);
char line[GPS_LINE_MAX];
int line_len = 0;

bool have_start = false;
bool lap_armed = false;
bool waiting_departure = false;
bool timing_active = false;
bool latest_fix = false;
double start_lat = 0.0;
double start_lon = 0.0;
double latest_lat = 0.0;
double latest_lon = 0.0;
uint32_t last_cross_ms = 0;
uint32_t last_fix_ms = 0;

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

double deg_min_to_decimal(const char *value, char hemi) {
    if (!value || !value[0]) return 0.0;
    const double raw = atof(value);
    const int deg = (int)(raw / 100.0);
    const double minutes = raw - (double)deg * 100.0;
    double decimal = (double)deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') decimal = -decimal;
    return decimal;
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

void update_lap(double lat, double lon) {
    const uint32_t now = millis();
    state.gps_fix_ok = true;
    latest_fix = true;
    latest_lat = lat;
    latest_lon = lon;
    last_fix_ms = now;

    if (!have_start) {
        return;
    }

    const float dist = distance_m(start_lat, start_lon, lat, lon);
    if (waiting_departure) {
        state.current_lap_ms = 0;
        if (dist >= START_RADIUS_M) {
            waiting_departure = false;
            timing_active = true;
            lap_armed = false;
            last_cross_ms = now;
        }
        return;
    }

    if (!timing_active || last_cross_ms == 0) return;

    state.current_lap_ms = now - last_cross_ms;

    if (dist >= REARM_RADIUS_M) {
        lap_armed = true;
    }

    if (lap_armed && dist <= START_RADIUS_M && now - last_cross_ms >= MIN_LAP_MS) {
        const uint32_t lap_ms = now - last_cross_ms;
        const uint8_t completed_lap = state.lap_count < 99 ? (uint8_t)(state.lap_count + 1) : 99;
        state.last_lap_ms = lap_ms;
        if (state.best_lap_ms == 0 || lap_ms < state.best_lap_ms) {
            state.best_lap_ms = lap_ms;
            state.best_lap_count = completed_lap;
        }
        state.current_lap_ms = 0;
        last_cross_ms = now;
        if (state.lap_count < 99) ++state.lap_count;
        lap_armed = false;
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
        if (line_len > 6) parse_rmc(line);
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
    gps_serial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

void poll() {
    while (gps_serial.available() > 0) {
        consume_char((char)gps_serial.read());
    }

    if (latest_fix && millis() - last_fix_ms > GPS_FIX_TIMEOUT_MS) {
        state.gps_fix_ok = false;
        latest_fix = false;
    }
}

bool start_at_current_fix() {
    if (!latest_fix) return false;

    start_lat = latest_lat;
    start_lon = latest_lon;
    have_start = true;
    lap_armed = false;
    waiting_departure = true;
    timing_active = false;
    last_cross_ms = 0;
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
    last_cross_ms = 0;
    state.current_lap_ms = 0;
}
}
