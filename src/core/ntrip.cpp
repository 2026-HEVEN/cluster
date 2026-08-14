#include "core/ntrip.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <cstring>
#include "core/gps_laptimer.h"
#include "ntrip_config.h"
#include "state.h"

namespace ntrip {
namespace {
constexpr uint32_t WIFI_RETRY_MS = 5000;
constexpr uint32_t NTRIP_RECONNECT_MS = 5000;
constexpr uint32_t NTRIP_CONNECT_TIMEOUT_MS = 1000;
constexpr uint32_t GGA_SEND_MS = 1000;
constexpr uint32_t GGA_MAX_AGE_MS = 2000;
constexpr uint32_t GGA_WARN_AGE_MS = 5000;
constexpr uint32_t RTCM_TIMEOUT_MS = 5000;
constexpr uint32_t STATUS_LOG_MS = 2000;
constexpr size_t RTCM_BUFFER_SIZE = 256;

WiFiClient client;
bool header_complete = false;
bool stream_ok = false;
String header_buffer;
uint32_t last_wifi_attempt_ms = 0;
uint32_t last_ntrip_attempt_ms = 0;
uint32_t last_gga_sent_ms = 0;
uint32_t last_status_log_ms = 0;
uint32_t stream_connected_ms = 0;
uint32_t total_rtcm_bytes = 0;
uint32_t last_rtcm_time_ms = 0;
bool wifi_connected_logged = false;
bool ntrip_connected_logged = false;
bool wifi_disconnected_logged = false;
bool gga_stale_logged = false;
bool rtcm_timeout_logged = false;

bool configured() {
    return ntrip_config::WIFI_SSID[0] != '\0' &&
           ntrip_config::HOST[0] != '\0' &&
           ntrip_config::MOUNTPOINT[0] != '\0';
}

String base64_encode(const char *text) {
    static const char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const size_t len = std::strlen(text);
    String out;
    out.reserve(((len + 2) / 3) * 4);

    for (size_t i = 0; i < len; i += 3) {
        const uint32_t a = (uint8_t)text[i];
        const uint32_t b = (i + 1 < len) ? (uint8_t)text[i + 1] : 0;
        const uint32_t c = (i + 2 < len) ? (uint8_t)text[i + 2] : 0;
        const uint32_t triple = (a << 16) | (b << 8) | c;

        out += alphabet[(triple >> 18) & 0x3F];
        out += alphabet[(triple >> 12) & 0x3F];
        out += (i + 1 < len) ? alphabet[(triple >> 6) & 0x3F] : '=';
        out += (i + 2 < len) ? alphabet[triple & 0x3F] : '=';
    }
    return out;
}

void connect_wifi(uint32_t now) {
    if (WiFi.status() == WL_CONNECTED) return;
    if (now - last_wifi_attempt_ms < WIFI_RETRY_MS) return;

    last_wifi_attempt_ms = now;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ntrip_config::WIFI_SSID, ntrip_config::WIFI_PASSWORD);
    Serial.print("[WIFI] Connecting to ");
    Serial.println(ntrip_config::WIFI_SSID);
}

void reset_stream_state() {
    header_buffer = "";
    header_complete = false;
    stream_ok = false;
    stream_connected_ms = 0;
    ntrip_connected_logged = false;
}

void close_stream() {
    client.stop();
    reset_stream_state();
}

void connect_ntrip(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (client.connected()) return;
    if (now - last_ntrip_attempt_ms < NTRIP_RECONNECT_MS) return;

    last_ntrip_attempt_ms = now;
    close_stream();

    Serial.print("[NTRIP] Connecting ");
    Serial.print(ntrip_config::HOST);
    Serial.print(":");
    Serial.println(ntrip_config::PORT);

    client.setTimeout(NTRIP_CONNECT_TIMEOUT_MS);
    if (!client.connect(ntrip_config::HOST, ntrip_config::PORT, NTRIP_CONNECT_TIMEOUT_MS)) {
        Serial.println("[NTRIP] Connection failed");
        Serial.println("[NTRIP] Retry in 5 sec");
        return;
    }

    String request = "GET /";
    request += ntrip_config::MOUNTPOINT;
    request += " HTTP/1.0\r\n";
    request += "User-Agent: NTRIP HEVEN-Cluster/1.0\r\n";
    request += "Accept: */*\r\n";
    request += "Connection: keep-alive\r\n";

    if (ntrip_config::USERNAME[0] != '\0') {
        String credentials = ntrip_config::USERNAME;
        credentials += ":";
        credentials += ntrip_config::PASSWORD;
        request += "Authorization: Basic ";
        request += base64_encode(credentials.c_str());
        request += "\r\n";
    }

    request += "\r\n";
    client.print(request);
}

bool header_has_success() {
    return header_buffer.indexOf("ICY 200") >= 0 ||
           header_buffer.indexOf("200 OK") >= 0;
}

void close_bad_stream() {
    Serial.print("[NTRIP] Bad caster response: ");
    Serial.println(header_buffer);
    Serial.println("[NTRIP] Retry in 5 sec");
    close_stream();
}

void process_header() {
    while (client.connected() && client.available() > 0 && !header_complete) {
        const char c = (char)client.read();
        header_buffer += c;

        const bool blank_line = header_buffer.indexOf("\r\n\r\n") >= 0;
        const bool icy_line = header_buffer.startsWith("ICY 200") && c == '\n';
        if (blank_line || icy_line) {
            header_complete = true;
            stream_ok = header_has_success();
            if (stream_ok) {
                stream_connected_ms = millis();
                rtcm_timeout_logged = false;
                if (!ntrip_connected_logged) {
                    Serial.println("[NTRIP] Connected");
                    Serial.print("[NTRIP] Mount point = ");
                    Serial.println(ntrip_config::MOUNTPOINT);
                    ntrip_connected_logged = true;
                }
            } else {
                close_bad_stream();
            }
            return;
        }

        if (header_buffer.length() > 512) {
            close_bad_stream();
            return;
        }
    }
}

void forward_rtcm(uint32_t now) {
    if (!stream_ok) return;

    uint8_t buffer[RTCM_BUFFER_SIZE];
    while (client.connected() && client.available() > 0) {
        const int n = client.read(buffer, sizeof(buffer));
        if (n <= 0) return;
        gps_laptimer::write_rtcm(buffer, (size_t)n);
        total_rtcm_bytes += (uint32_t)n;
        last_rtcm_time_ms = now;
        rtcm_timeout_logged = false;
    }
}

void send_gga(uint32_t now) {
    if (!stream_ok || now - last_gga_sent_ms < GGA_SEND_MS) return;

    const char *gga = gps_laptimer::last_gga_sentence();
    if (!gga || gga[0] == '\0') return;
    const uint32_t gga_ms = gps_laptimer::last_gga_ms();
    if (gga_ms == 0 || now - gga_ms > GGA_MAX_AGE_MS) return;

    client.write((const uint8_t *)gga, std::strlen(gga));
    client.write((const uint8_t *)"\r\n", 2);
    last_gga_sent_ms = now;
    gga_stale_logged = false;
    Serial.println("[NTRIP] GGA sent");
}

void check_timeouts(uint32_t now) {
    const uint32_t gga_ms = gps_laptimer::last_gga_ms();
    if (gga_ms != 0 && now - gga_ms > GGA_WARN_AGE_MS) {
        if (!gga_stale_logged) {
            Serial.println("[GPS] WARNING: GGA stale");
            gga_stale_logged = true;
        }
    } else {
        gga_stale_logged = false;
    }

    if (!stream_ok) return;
    const bool rtcm_missing = last_rtcm_time_ms == 0
        ? (stream_connected_ms != 0 && now - stream_connected_ms > RTCM_TIMEOUT_MS)
        : (now - last_rtcm_time_ms > RTCM_TIMEOUT_MS);
    if (rtcm_missing) {
        if (!rtcm_timeout_logged) {
            Serial.println("[NTRIP] WARNING: RTCM timeout");
            Serial.println("[NTRIP] Retry in 5 sec");
            rtcm_timeout_logged = true;
        }
        close_stream();
    }
}

void log_status(uint32_t now) {
    if (now - last_status_log_ms < STATUS_LOG_MS) return;
    last_status_log_ms = now;

    Serial.println("========== RTK STATUS ==========");
    Serial.print("WiFi       : ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.print("NTRIP      : ");
    Serial.println(stream_ok ? "CONNECTED" : "DISCONNECTED");
    Serial.print("Mount      : ");
    Serial.println(ntrip_config::MOUNTPOINT);
    Serial.println("GPS UART   : 115200");
    Serial.print("GPS Fix    : ");
    Serial.println(state.gps_fix_ok ? "3D" : (state.gps_data_ok ? "SEARCH" : "NO FIX"));
    Serial.print("RTK        : ");
    Serial.println(gps_laptimer::rtk_status_label());
    Serial.print("Satellites : ");
    if (gps_laptimer::satellites() == 0) Serial.println("---");
    else Serial.println(gps_laptimer::satellites());
    Serial.print("HDOP       : ");
    if (gps_laptimer::hdop() <= 0.0f) Serial.println("---");
    else Serial.println(gps_laptimer::hdop(), 1);
    Serial.print("GGA age    : ");
    if (gps_laptimer::last_gga_ms() == 0) Serial.println("---");
    else Serial.println((now - gps_laptimer::last_gga_ms()) / 1000.0f, 1);
    Serial.print("RTCM bytes : ");
    Serial.println(total_rtcm_bytes);
    Serial.print("RTCM age   : ");
    if (last_rtcm_time_ms == 0) Serial.println("---");
    else Serial.println((now - last_rtcm_time_ms) / 1000.0f, 1);
    Serial.println("================================");
}
}

void begin() {
    if (!configured()) {
        Serial.println("[NTRIP] disabled: create include/ntrip_secrets.h");
        return;
    }

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ntrip_config::WIFI_SSID, ntrip_config::WIFI_PASSWORD);
    last_wifi_attempt_ms = millis();
    Serial.print("[WIFI] Connecting to ");
    Serial.println(ntrip_config::WIFI_SSID);
}

void poll() {
    if (!configured()) return;

    const uint32_t now = millis();
    connect_wifi(now);
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_connected_logged) {
            Serial.println("[WIFI] Connected");
            Serial.print("[WIFI] IP = ");
            Serial.println(WiFi.localIP());
            wifi_connected_logged = true;
            wifi_disconnected_logged = false;
        }
    } else {
        if (wifi_connected_logged || !wifi_disconnected_logged) {
            Serial.println("[WIFI] Disconnected");
            Serial.println("[WIFI] Reconnecting...");
            wifi_disconnected_logged = true;
        }
        close_stream();
        wifi_connected_logged = false;
    }
    connect_ntrip(now);

    if (!client.connected()) {
        reset_stream_state();
    } else if (!header_complete) {
        process_header();
    }

    forward_rtcm(now);
    send_gga(now);
    check_timeouts(now);
    log_status(now);
}

bool wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}

bool connected() {
    return stream_ok && client.connected();
}

uint32_t rtcm_bytes() {
    return total_rtcm_bytes;
}

uint32_t last_rtcm_ms() {
    return last_rtcm_time_ms;
}
}

