#include <unity.h>
#include "framebuffer.h"
#include "modules/widgets/widget_warnings.h"

static int lit_in(FrameBuffer &fb, int x0, int y0, int w, int h) {
    int n = 0; for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) if (fb.get(x, y)) n++;
    return n;
}
void test_clear_draws_labels_and_empty_boxes(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 0, false, false);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 34, 12) > 0);
    TEST_ASSERT_TRUE(lit_in(fb, 24, 0, 10, 10) < 100);
}
void test_fault_does_not_draw_warn_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 0, true, false);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 13, 42, 11) == 0);
}
void test_regen_fills_regen_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 0, false, true);
    TEST_ASSERT_EQUAL_INT(100, lit_in(fb, 24, 0, 10, 10));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clear_draws_labels_and_empty_boxes);
    RUN_TEST(test_fault_does_not_draw_warn_box);
    RUN_TEST(test_regen_fills_regen_box);
    return UNITY_END();
}
