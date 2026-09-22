// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/display_blit.h"
#include "core/board_pins.h"
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

namespace {
constexpr bool PANEL_INVERTED = true; // Devicemart 3.2-inch THL/CD01 ILI9341 panel colors.
Adafruit_ILI9341 tft(board_pins::LCD_CS, board_pins::LCD_DC, board_pins::LCD_RST);
}

namespace display_blit {

void begin() {
    SPI.begin(board_pins::SPI_SCK, board_pins::SPI_MISO,
              board_pins::SPI_MOSI, board_pins::LCD_CS);
    tft.begin();
    tft.setRotation(1);       // landscape: 320x240
    tft.invertDisplay(PANEL_INVERTED);
    tft.fillScreen(ILI9341_BLACK);
}

void show(const FrameBuffer &fb) {
    show(fb, false);
}

void show(const FrameBuffer &fb, bool warning_tint) {
    static uint16_t line[FB_W];
    const uint16_t on_color = ILI9341_WHITE;
    const uint16_t off_color = warning_tint ? ILI9341_RED : ILI9341_BLACK;
    for (int y = 0; y < FB_H; ++y) {
        for (int x = 0; x < FB_W; ++x) {
            line[x] = fb.get(x, y) ? on_color : off_color;
        }
        tft.drawRGBBitmap(0, y, line, FB_W, 1);
    }
}

} // namespace display_blit
