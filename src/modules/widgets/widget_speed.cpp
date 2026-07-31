// [FILL-IN] Edit this file. Draw your widget into the framebuffer.
#include "modules/widgets/widget_speed.h"

namespace {
int digit_count(int value) {
    if (value < 0) value = -value;
    int digits = 1;
    while (value >= 10) {
        value /= 10;
        ++digits;
    }
    return digits;
}

int speed_scale(int kph) {
    const int digits = digit_count(kph);
    if (digits <= 1) return 16;
    if (digits == 2) return 15;
    return 12;
}
}

void widget_speed_draw(FrameBuffer &fb, int x, int y, int kph) {
    if (kph < 0) kph = 0;
    if (kph > 999) kph = 999;
    fb_number(fb, x, y, kph, speed_scale(kph));
}
