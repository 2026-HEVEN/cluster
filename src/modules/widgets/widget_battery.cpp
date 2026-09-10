// [FILL-IN] Edit this file. Draw your widget into the framebuffer.
#include "modules/widgets/widget_battery.h"
#include <cstdio>
#include <cstring>

void widget_battery_draw(FrameBuffer &fb, int x, int y, int soc_pct) {
    constexpr int gauge_w = 14;
    constexpr int gauge_h = 64;
    constexpr int label_y_offset = 72;
    char label[8];

    if (soc_pct < 0) {
        fb_rect(fb, x, y, gauge_w, gauge_h, false, true);
        fb_text(fb, x - 10, y + label_y_offset, "--%", 2);
        return;
    }

    if (soc_pct > 100) soc_pct = 100;

    fb_rect(fb, x, y, gauge_w, gauge_h, false, true);
    const int fillh = (gauge_h - 2) * soc_pct / 100;
    if (fillh > 0) {
        fb_rect(fb, x + 1, y + gauge_h - 1 - fillh, gauge_w - 2, fillh, true, true);
    }
    std::snprintf(label, sizeof(label), "%d%%", soc_pct);
    fb_text(fb, x + 7 - ((int)std::strlen(label) * 12 - 2) / 2,
            y + label_y_offset, label, 2);
}

void widget_energy_draw(FrameBuffer &fb, int x, int y, int soc_pct,
                        int16_t hv_decivolts, int16_t lv_centivolts, bool fresh) {
    fb_text(fb, x + 19, y, "HV SOC", 1);
    widget_battery_draw(fb, x + 29, y + 12, soc_pct);
    char text[20];
    const auto voltage = [&](int row, const char *name, int32_t raw, int divisor) {
        fb_text(fb, x, y + row, name, 1);
        if (!fresh) {
            std::snprintf(text, sizeof(text), "-- V");
        } else {
            const int32_t magnitude = raw < 0 ? -raw : raw;
            std::snprintf(text, sizeof(text), "%s%ld.%0*ldV",
                          raw < 0 ? "-" : "", (long)(magnitude / divisor),
                          divisor == 100 ? 2 : 1, (long)(magnitude % divisor));
        }
        const int scale = std::strlen(text) * 12 - 2 <= 74 ? 2 : 1;
        fb_text(fb, x, y + row + 11, text, scale);
    };
    voltage(108, "HV", hv_decivolts, 10);
    voltage(147, "LV", lv_centivolts, 100);
}
