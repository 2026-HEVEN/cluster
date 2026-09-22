#pragma once

// Physical Cluster PCB V3 pin map.
// Source of truth: Notion "PCB V3 설계" (2026-09-17).
namespace board_pins {
constexpr int LCD_CS = 23;
constexpr int LCD_RST = 22;
constexpr int LCD_DC = 21;
constexpr int SPI_MOSI = 17;
constexpr int SPI_SCK = 18;
constexpr int SPI_MISO = 35;
constexpr int TOUCH_CS = 16;

constexpr int CAN_RX = 14;
constexpr int CAN_TX = 13;

constexpr int GNSS_RX = 25;
constexpr int GNSS_TX = 26;
constexpr int GNSS_PPS = 27;

constexpr int VESS_PWM = 4;
constexpr int TV_SWITCH = 32;
constexpr int PADDOCK_SWITCH = 33;
constexpr int HOME_BUTTON = 19;
constexpr int LAP_BUTTON = 5;
constexpr int REGEN_BIT0 = 36;
constexpr int REGEN_BIT1 = 39;
constexpr int START_SENSE_ADC = 34;
} // namespace board_pins
