#pragma once
#include "framebuffer.h"
// [FILL-IN] Draws the speed number only at (x,y). Pure: fb only, no state/Arduino.

void widget_speed_draw(FrameBuffer &fb, int x, int y, int speed_kph);
