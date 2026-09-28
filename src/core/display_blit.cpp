// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/display_blit.h"
#include "core/board_pins.h"
#include <Arduino.h>
#include <SPI.h>
#include <cstring>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

namespace {
constexpr bool PANEL_INVERTED = true; // Devicemart 3.2-inch THL/CD01 ILI9341 panel colors.
Adafruit_ILI9341 tft(board_pins::LCD_CS, board_pins::LCD_DC, board_pins::LCD_RST);
constexpr int IDLE_HOOK_ROWS = 24;
// Last frame pushed to the panel; only rows that differ are sent again.
// A full frame takes ~60 ms, a typical update (a few changing numbers) a few ms.
uint8_t shown[FB_STRIDE * FB_H];
bool shown_valid = false;
bool shown_tint = false;
void (*idle_hook)() = nullptr;
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
    const bool full = !shown_valid || warning_tint != shown_tint;
    int sent = 0;
    for (int y = 0; y < FB_H; ++y) {
        const uint8_t *row = fb.bits + y * FB_STRIDE;
        uint8_t *prev = shown + y * FB_STRIDE;
        if (!full && std::memcmp(row, prev, FB_STRIDE) == 0) continue;
        std::memcpy(prev, row, FB_STRIDE);
        for (int x = 0; x < FB_W; ++x) {
            line[x] = fb.get(x, y) ? on_color : off_color;
        }
        tft.drawRGBBitmap(0, y, line, FB_W, 1);
        if (idle_hook && ++sent % IDLE_HOOK_ROWS == 0) idle_hook();
    }
    shown_valid = true;
    shown_tint = warning_tint;
}

void set_idle_hook(void (*hook)()) {
    idle_hook = hook;
}

} // namespace display_blit
