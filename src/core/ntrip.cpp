#include "core/ntrip.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <cstring>
#include "core/gps_laptimer.h"
#include "ntrip_config.h"

namespace ntrip {
namespace {
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t NTRIP_RECONNECT_MS = 5000;
constexpr uint32_t GGA_SEND_MS = 5000;
constexpr uint32_t STATUS_LOG_MS = 1000;
constexpr size_t RTCM_BUFFER_SIZE = 256;

WiFiClient client;
bool header_complete = false;
bool stream_ok = false;
String header_buffer;
uint32_t last_wifi_attempt_ms = 0;
uint32_t last_ntrip_attempt_ms = 0;
uint32_t last_gga_sent_ms = 0;
uint32_t last_status_log_ms = 0;
uint32_t total_rtcm_bytes = 0;
uint32_t last_rtcm_time_ms = 0;
bool wifi_connected_logged = false;
bool ntrip_connected_logged = false;

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
    Serial.print("[NTRIP] Wi-Fi connecting to ");
    Serial.println(ntrip_config::WIFI_SSID);
}

void connect_ntrip(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (client.connected()) return;
    if (now - last_ntrip_attempt_ms < NTRIP_RECONNECT_MS) return;

    last_ntrip_attempt_ms = now;
    client.stop();
    header_buffer = "";
    header_complete = false;
    stream_ok = false;
    ntrip_connected_logged = false;

    Serial.print("[NTRIP] Connecting ");
    Serial.print(ntrip_config::HOST);
    Serial.print(":");
    Serial.println(ntrip_config::PORT);

    if (!client.connect(ntrip_config::HOST, ntrip_config::PORT)) {
        Serial.println("[NTRIP] Caster connection failed");
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
    client.stop();
    header_complete = false;
    stream_ok = false;
    header_buffer = "";
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
                if (!ntrip_connected_logged) {
                    Serial.println("[NTRIP] NTRIP connected");
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
    }
}

void send_gga(uint32_t now) {
    if (!stream_ok || now - last_gga_sent_ms < GGA_SEND_MS) return;

    const char *gga = gps_laptimer::last_gga_sentence();
    if (!gga || gga[0] == '\0') return;

    client.write((const uint8_t *)gga, std::strlen(gga));
    client.write((const uint8_t *)"\r\n", 2);
    last_gga_sent_ms = now;
}

void log_status(uint32_t now) {
    if (now - last_status_log_ms < STATUS_LOG_MS) return;
    last_status_log_ms = now;

    Serial.print("[NTRIP] WiFi=");
    Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "WAIT");
    Serial.print(" NTRIP=");
    Serial.print(stream_ok ? "OK" : "WAIT");
    Serial.print(" RTCM received bytes=");
    Serial.print(total_rtcm_bytes);
    Serial.print(" Last RTCM age=");
    if (last_rtcm_time_ms == 0) Serial.print("---");
    else Serial.print(now - last_rtcm_time_ms);
    Serial.print("ms GPS NMEA receiving=");
    if (gps_laptimer::last_nmea_ms() == 0) Serial.print("WAIT");
    else Serial.print(now - gps_laptimer::last_nmea_ms());
    Serial.print("ms PPS=");
    if (gps_laptimer::last_pps_ms() == 0) Serial.print("WAIT");
    else Serial.print(now - gps_laptimer::last_pps_ms());
    Serial.print("ms RTK=");
    Serial.println(gps_laptimer::rtk_status_label());
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
    Serial.print("[NTRIP] Wi-Fi begin ");
    Serial.println(ntrip_config::WIFI_SSID);
}

void poll() {
    if (!configured()) return;

    const uint32_t now = millis();
    connect_wifi(now);
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_connected_logged) {
            Serial.println("[NTRIP] Wi-Fi connected");
            wifi_connected_logged = true;
        }
    } else {
        wifi_connected_logged = false;
        ntrip_connected_logged = false;
    }
    connect_ntrip(now);

    if (!client.connected()) {
        stream_ok = false;
        header_complete = false;
        ntrip_connected_logged = false;
    } else if (!header_complete) {
        process_header();
    }

    forward_rtcm(now);
    send_gga(now);
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

