// Not a correctness test -- a visual aid. Renders the same widget layout as
// src/core/app_wiring.cpp's display_update() to a 24-bit BMP so anyone
// (Windows/macOS/Linux, no image tool needed) can open it and see the
// per-widget allocated space. Run: pio test -e native -f test_render_layout
#include <unity.h>
#include "framebuffer.h"
#include "modules/widgets/widget_speed.h"
#include "modules/widgets/widget_battery.h"
#include "modules/widgets/widget_warnings.h"
#include "modules/widgets/widget_gear.h"
#include "modules/widgets/widget_laptime.h"
#include <cstdio>
#include <cstdint>
#include <vector>
#include <filesystem>

namespace {

void write_bmp(const char *path, int w, int h, const std::vector<uint8_t> &rgb) {
    int row_stride = w * 3;
    int pad = (4 - (row_stride % 4)) % 4;
    int data_size = (row_stride + pad) * h;
    int file_size = 54 + data_size;

    uint8_t header[54] = {0};
    header[0] = 'B'; header[1] = 'M';
    *(int32_t *)&header[2]  = file_size;
    *(int32_t *)&header[10] = 54;
    *(int32_t *)&header[14] = 40;
    *(int32_t *)&header[18] = w;
    *(int32_t *)&header[22] = h;
    *(int16_t *)&header[26] = 1;
    *(int16_t *)&header[28] = 24;
    *(int32_t *)&header[34] = data_size;

    std::FILE *f = std::fopen(path, "wb");
    std::fwrite(header, 1, 54, f);
    uint8_t padbuf[3] = {0, 0, 0};
    for (int y = h - 1; y >= 0; y--) {
        const uint8_t *row = &rgb[y * row_stride];
        for (int x = 0; x < w; x++) {
            uint8_t px[3] = { row[x * 3 + 2], row[x * 3 + 1], row[x * 3 + 0] };
            std::fwrite(px, 1, 3, f);
        }
        if (pad) std::fwrite(padbuf, 1, pad, f);
    }
    std::fclose(f);
}

void render_framebuffer(std::vector<uint8_t> &rgb, const FrameBuffer &fb,
                        int scale, bool warning_screen) {
    const int W = FB_W * scale;
    const int H = FB_H * scale;
    rgb.assign((size_t)W * H * 3, 0);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const bool on = fb.get(x / scale, y / scale);
            const size_t i = ((size_t)y * W + x) * 3;
            if (on) {
                rgb[i] = rgb[i + 1] = rgb[i + 2] = 255;
            } else if (warning_screen) {
                rgb[i] = 255;
                rgb[i + 1] = 0;
                rgb[i + 2] = 0;
            }
        }
    }
}

void draw_normal_layout(FrameBuffer &fb, bool warning) {
    fb.clear();
    widget_speed_draw(fb,    10,  10, 160);
    widget_warnings_draw(fb, 248,  22, warning, true);
    widget_gear_draw(fb,     289,  16, 2 /* D */);
    widget_battery_draw(fb, 285,  48, -1);
    widget_laptime_draw(fb,  10, 136, 3, 85670, true);
    widget_best_lap_draw(fb, 205, 207, 1, 80770);
}

void draw_warning_detail(FrameBuffer &fb) {
    fb.clear();
    fb_text(fb, 18, 14, "WARNING", 5);
    fb_text(fb, 18, 112, "MOTOR HOT", 5);
}

const uint8_t *status_glyph(char c) {
    static const uint8_t glyph_b[7] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
    static const uint8_t glyph_s[7] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
    if (c == 'B') return glyph_b;
    if (c == 'S') return glyph_s;
    return font_glyph(c);
}

void status_text(FrameBuffer &fb, int x, int y, const char *text, int scale) {
    if (scale < 1) scale = 1;
    int cx = x;
    for (const char *p = text; *p; ++p) {
        const uint8_t *g = status_glyph(*p);
        if (g) {
            for (int r = 0; r < 7; ++r) {
                for (int c = 0; c < 5; ++c) {
                    if (g[r] & (0x10 >> c)) {
                        fb_rect(fb, cx + c * scale, y + r * scale,
                                scale, scale, true, true);
                    }
                }
            }
        }
        cx += 6 * scale;
    }
}

void status_line(FrameBuffer &fb, int &y, const char *text, int scale) {
    status_text(fb, 8, y, text, scale);
    y += scale * 8 + 1;
}

void draw_status_detail(FrameBuffer &fb) {
    fb.clear();
    fb_text(fb, 8, 4, "CAR CHECK", 3);
    int y = 31;
    status_line(fb, y, "CAN L OK R OK", 2);
    status_line(fb, y, "VCU OK HV ON", 2);
    status_line(fb, y, "BMS OK 078% 51V", 2);

    y += 4;
    status_line(fb, y, "LEFT FAULT", 3);
    status_line(fb, y, "MTR 088C HOT", 2);
    status_line(fb, y, "CTRL 074C HOT", 2);
    status_line(fb, y, "VOLT 121.5 OVER", 2);

    y += 5;
    status_line(fb, y, "RIGHT OK", 3);
    status_line(fb, y, "MTR 052C OK", 2);
    status_line(fb, y, "CTRL 048C OK", 2);
    status_line(fb, y, "VOLT 119.8 OK", 2);
}

void write_frame(const char *path, FrameBuffer &fb, int scale, bool warning_screen) {
    std::vector<uint8_t> rgb;
    render_framebuffer(rgb, fb, scale, warning_screen);
    write_bmp(path, FB_W * scale, FB_H * scale, rgb);
    std::FILE *check = std::fopen(path, "rb");
    TEST_ASSERT_NOT_NULL_MESSAGE(check, path);
    if (check) std::fclose(check);
    auto full_path = std::filesystem::absolute(path);
    TEST_MESSAGE(full_path.string().c_str());
}

}  // namespace

void test_render_layout_writes_bmp(void) {
    FrameBuffer fb;
    const int SCALE = 3;

    draw_normal_layout(fb, false);
    write_frame("render_layout_current.bmp", fb, SCALE, false);

    draw_normal_layout(fb, true);
    write_frame("render_layout_warning.bmp", fb, SCALE, true);

    draw_warning_detail(fb);
    write_frame("render_warning_detail_motor_hot.bmp", fb, SCALE, true);

    draw_status_detail(fb);
    write_frame("render_status_detail.bmp", fb, SCALE, false);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_render_layout_writes_bmp);
    return UNITY_END();
}
