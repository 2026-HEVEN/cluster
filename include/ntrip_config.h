#pragma once
#include <cstdint>

#if __has_include("ntrip_secrets.h")
#include "ntrip_secrets.h"
#endif

#ifndef NTRIP_WIFI_SSID
#define NTRIP_WIFI_SSID ""
#endif

#ifndef NTRIP_WIFI_PASSWORD
#define NTRIP_WIFI_PASSWORD ""
#endif

#ifndef NTRIP_HOST
#define NTRIP_HOST ""
#endif

#ifndef NTRIP_PORT
#define NTRIP_PORT 2101
#endif

#ifndef NTRIP_MOUNTPOINT
#define NTRIP_MOUNTPOINT ""
#endif

#ifndef NTRIP_USERNAME
#define NTRIP_USERNAME ""
#endif

#ifndef NTRIP_PASSWORD
#define NTRIP_PASSWORD ""
#endif

namespace ntrip_config {
constexpr char WIFI_SSID[] = NTRIP_WIFI_SSID;
constexpr char WIFI_PASSWORD[] = NTRIP_WIFI_PASSWORD;
constexpr char HOST[] = NTRIP_HOST;
constexpr uint16_t PORT = NTRIP_PORT;
constexpr char MOUNTPOINT[] = NTRIP_MOUNTPOINT;
constexpr char USERNAME[] = NTRIP_USERNAME;
constexpr char PASSWORD[] = NTRIP_PASSWORD;
}
