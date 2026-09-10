#pragma once
#include "framebuffer.h"
// [FILL-IN] Draws a SOC bar + percentage at (x,y). Pure.

void widget_battery_draw(FrameBuffer &fb, int x, int y, int soc_pct);
// 74 x 172 pixel region: compact SOC gauge followed by measured bus voltages.
void widget_energy_draw(FrameBuffer &fb, int x, int y, int soc_pct,
                        int16_t hv_decivolts, int16_t lv_centivolts, bool fresh);
