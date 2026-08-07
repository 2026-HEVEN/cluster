#pragma once
#include "framebuffer.h"
// [FILL-IN] Draws the regen indicator at (x,y). Faults are shown by the red
// warning screen instead of a small WARN label.

void widget_warnings_draw(FrameBuffer &fb, int x, int y, bool fault,
                          bool regen_auto);
