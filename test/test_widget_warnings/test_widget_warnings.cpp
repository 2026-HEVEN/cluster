#include <unity.h>
#include "framebuffer.h"
#include "modules/widgets/widget_warnings.h"

static int lit_in(FrameBuffer &fb, int x0, int y0, int w, int h) {
    int n = 0; for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) if (fb.get(x, y)) n++;
    return n;
}
void test_clear_draws_labels_and_empty_boxes(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, false, 0);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 0, 34, 24) > 0);
    TEST_ASSERT_EQUAL_INT(0, lit_in(fb, 28, 15, 12, 12));
    TEST_ASSERT_TRUE(lit_in(fb, 24, 0, 10, 10) < 100);
}
void test_fault_does_not_draw_warn_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, true, false, 0);
    FrameBuffer normal; normal.clear();
    widget_warnings_draw(normal, 0, 14, false, false, 0);
    TEST_ASSERT_EQUAL_MEMORY(normal.bits, fb.bits, sizeof(fb.bits));
}
void test_hv_fills_hv_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, true, 0);
    TEST_ASSERT_EQUAL_INT(196, lit_in(fb, 27, 14, 14, 14));
}
void test_regen_off_draws_empty_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, false, 0);
    TEST_ASSERT_TRUE(lit_in(fb, 24, 0, 10, 10) < 100);
}
void test_regen_on_fills_box(void) {
    for (uint8_t level = 1; level <= 3; ++level) {
        FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, false, level);
        TEST_ASSERT_EQUAL_INT(100, lit_in(fb, 24, 0, 10, 10));
    }
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clear_draws_labels_and_empty_boxes);
    RUN_TEST(test_fault_does_not_draw_warn_box);
    RUN_TEST(test_hv_fills_hv_box);
    RUN_TEST(test_regen_off_draws_empty_box);
    RUN_TEST(test_regen_on_fills_box);
    return UNITY_END();
}
