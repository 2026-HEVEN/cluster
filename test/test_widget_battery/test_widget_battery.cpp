#include <unity.h>
#include "framebuffer.h"
#include "modules/widgets/widget_battery.h"

static int lit_in(FrameBuffer &fb, int x0, int y0, int w, int h) {
    int n = 0; for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) if (fb.get(x, y)) n++;
    return n;
}
void test_full_more_lit_than_empty(void) {
    FrameBuffer full; full.clear(); widget_battery_draw(full, 0, 0, 100);
    FrameBuffer empty; empty.clear(); widget_battery_draw(empty, 0, 0, 0);
    TEST_ASSERT_TRUE(lit_in(full, 0, 0, 30, 110) > lit_in(empty, 0, 0, 30, 110));
}

void test_unknown_soc_draws_outline(void) {
    FrameBuffer fb; fb.clear(); widget_battery_draw(fb, 0, 0, -1);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 30, 110) > 0);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_full_more_lit_than_empty);
    RUN_TEST(test_unknown_soc_draws_outline);
    return UNITY_END();
}
