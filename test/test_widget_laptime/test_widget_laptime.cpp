#include <unity.h>
#include "framebuffer.h"
#include "modules/widgets/widget_laptime.h"

static int lit_in(FrameBuffer &fb, int x0, int y0, int w, int h) {
    int n = 0;
    for (int y = y0; y < y0 + h; ++y)
        for (int x = x0; x < x0 + w; ++x)
            if (fb.get(x, y)) ++n;
    return n;
}

void test_laptime_draws(void) {
    FrameBuffer fb;
    fb.clear();
    widget_laptime_draw(fb, 0, 0, 3, 83420, true);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 200, 60) > 0);
    TEST_ASSERT_TRUE(lit_in(fb, 95, 0, 100, 16) > 0);
}

void test_laptime_waiting_still_draws_placeholder(void) {
    FrameBuffer fb;
    fb.clear();
    widget_laptime_draw(fb, 0, 0, 0, 99999, false);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 200, 60) > 0);
    TEST_ASSERT_TRUE(lit_in(fb, 95, 0, 100, 16) > 0);
}

void test_best_lap_draws_record(void) {
    FrameBuffer fb;
    fb.clear();
    widget_best_lap_draw(fb, 0, 0, 1, 80770);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 100, 30) > 0);
}

void test_best_lap_draws_placeholder(void) {
    FrameBuffer fb;
    fb.clear();
    widget_best_lap_draw(fb, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 100, 30) > 0);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_laptime_draws);
    RUN_TEST(test_laptime_waiting_still_draws_placeholder);
    RUN_TEST(test_best_lap_draws_record);
    RUN_TEST(test_best_lap_draws_placeholder);
    return UNITY_END();
}
