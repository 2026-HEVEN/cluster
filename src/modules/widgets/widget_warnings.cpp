// [FILL-IN] Edit this file. Draw your widget into the framebuffer.
#include "modules/widgets/widget_warnings.h"
#include <cstdint>

namespace {
void indicator_box(FrameBuffer &fb, int x, int y, bool active) {
    fb_rect(fb, x, y, 10, 10, active, true);
}
}

void widget_warnings_draw(FrameBuffer &fb, int x, int y, bool fault, bool hv,
                          uint8_t regen_level) {
    (void)fault;
    if (regen_level > 3) regen_level = 3;
    fb_text(fb, x, y - 14, "RGN", 1);
    const char regen_text[2] = {(char)('0' + regen_level), '\0'};
    fb_text(fb, x + 24, y - 14, regen_text, 1);

    fb_text(fb, x, y, "HV", 1);
    indicator_box(fb, x + 18, y, hv);
}
