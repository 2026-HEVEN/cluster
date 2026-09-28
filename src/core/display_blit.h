// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "framebuffer.h"
// [LOCKED] Pushes the 1bpp framebuffer to the ILI9341 panel.

namespace display_blit {
    void begin();
    void show(const FrameBuffer &fb);
    void show(const FrameBuffer &fb, bool warning_tint);
    // Called every few rows while a large update is being pushed, so inputs
    // (touch) keep being sampled during a full-screen redraw.
    void set_idle_hook(void (*hook)());
}
