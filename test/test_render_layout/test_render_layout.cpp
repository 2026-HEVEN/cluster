// Not a correctness test -- a visual aid. Renders the same widget layout as
// src/core/app_wiring.cpp's display_update() to a 24-bit BMP so anyone
// (Windows/macOS/Linux, no image tool needed) can open it and see the
// per-widget allocated space. Run: pio test -e native -f test_render_layout
#include <unity.h>
#include "framebuffer.h"
#include "modules/diagnostics.h"
#include "modules/widgets/widget_speed.h"
#include "modules/widgets/widget_battery.h"
#include "modules/widgets/widget_warnings.h"
#include "modules/widgets/widget_gear.h"
#include "modules/widgets/widget_laptime.h"
#include <cstdio>
#include <cstdint>
#include <vector>

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

void draw_normal_layout(FrameBuffer &fb, bool warning, int soc = 78,
                        int16_t hv = 537, int16_t lv = 1342, bool fresh = true) {
    fb.clear();
    widget_speed_draw(fb,    10,  18, 160);
    widget_gear_draw(fb,     270,   8, 2 /* D */);
    widget_energy_draw(fb, 244, 48, soc, hv, lv, fresh);
    widget_laptime_draw(fb,  18, 171, 3, 85670, true);
    widget_best_lap_draw(fb, 18, 199, 1, 80770);
    check_home_nav(fb, warning);
}

void write_frame(const char *path, FrameBuffer &fb, int scale, bool warning_screen) {
    std::vector<uint8_t> rgb;
    render_framebuffer(rgb, fb, scale, warning_screen);
    write_bmp(path, FB_W * scale, FB_H * scale, rgb);
    std::FILE *check = std::fopen(path, "rb");
    TEST_ASSERT_NOT_NULL_MESSAGE(check, path);
    if (check) std::fclose(check);
    TEST_MESSAGE(path);
}

}  // namespace

void test_render_layout_writes_bmp(void) {
    FrameBuffer fb;
    const int SCALE = 3;

    draw_normal_layout(fb, false);
    write_frame("render_layout_current.bmp", fb, SCALE, false);

    draw_normal_layout(fb, true);
    write_frame("render_layout_warning.bmp", fb, SCALE, true);

    draw_normal_layout(fb, false, -1, 0, 0, false);
    write_frame("render_energy_unknown.bmp", fb, SCALE, false);

    draw_normal_layout(fb, false, 100, -32768, -32768, true);
    write_frame("render_energy_limits.bmp", fb, SCALE, false);

    // Car Check/graph previews are rendered by test_diagnostics using production code.
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_render_layout_writes_bmp);
    return UNITY_END();
}
