// [FILL-IN] Edit this file. Draw your widget into the framebuffer.
#include "modules/widgets/widget_gear.h"

void widget_gear_draw(FrameBuffer &fb, int x, int y, int gear) {
    const char *label;
    switch (gear) {
        case 0: label = "N"; break;
        case 1: label = "R"; break;
        case 2: label = "D"; break;
        case 3: label = "P"; break;
        default: return;   // unknown -> blank
    }
    fb_text(fb, x, y, label, 5);
}
