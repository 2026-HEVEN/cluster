// [FILL-IN] Edit this file. Draw your widget into the framebuffer.
#include "modules/widgets/widget_battery.h"
#include <cstdio>

void widget_battery_draw(FrameBuffer &fb, int x, int y, int soc_pct) {
    constexpr int gauge_w = 30;
    constexpr int gauge_h = 110;
    constexpr int label_y_offset = 121;
    char label[8];

    if (soc_pct < 0) {
        fb_rect(fb, x, y, gauge_w, gauge_h, false, true);
        fb_text(fb, x - 3, y + label_y_offset, "--%", 2);
        return;
    }

    if (soc_pct > 100) soc_pct = 100;

    fb_rect(fb, x, y, gauge_w, gauge_h, false, true);
    const int fillh = (gauge_h - 2) * soc_pct / 100;
    if (fillh > 0) {
        fb_rect(fb, x + 1, y + gauge_h - 1 - fillh, gauge_w - 2, fillh, true, true);
    }
    std::snprintf(label, sizeof(label), "%d%%", soc_pct);
    fb_text(fb, x - 3, y + label_y_offset, label, 2);
}
