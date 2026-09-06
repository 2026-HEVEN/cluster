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
    TEST_ASSERT_TRUE(lit_in(fb, 18, 14, 10, 10) < 100);
    TEST_ASSERT_TRUE(lit_in(fb, 24, 0, 10, 10) < 100);
}
void test_fault_does_not_draw_warn_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, true, false, 0);
    TEST_ASSERT_TRUE(lit_in(fb, 0, 27, 42, 11) == 0);
}
void test_hv_fills_hv_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, true, 0);
    TEST_ASSERT_EQUAL_INT(100, lit_in(fb, 18, 14, 10, 10));
}
void test_regen_off_draws_empty_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, false, 1);
    TEST_ASSERT_TRUE(lit_in(fb, 24, 0, 10, 10) < 100);
}
void test_regen_on_fills_box(void) {
    FrameBuffer fb; fb.clear(); widget_warnings_draw(fb, 0, 14, false, false, 2);
    TEST_ASSERT_EQUAL_INT(100, lit_in(fb, 24, 0, 10, 10));
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
