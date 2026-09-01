#pragma once
#include "framebuffer.h"
#include <cstdint>
// [FILL-IN] Draws the regen and HV indicators at (x,y). Faults are shown by the red
// warning screen instead of a small WARN label.

void widget_warnings_draw(FrameBuffer &fb, int x, int y, bool fault, bool hv,
                          uint8_t regen_level);
