#include <unity.h>
#include "framebuffer.h"

void test_glyph_zero_top_row(void) {
    // '0' row0 = 0x0E = 01110 -> cols 1,2,3 on; cols 0,4 off (bit4 = leftmost).
    FrameBuffer fb; fb.clear();
    fb_text(fb, 0, 0, "0", 1);
    TEST_ASSERT_FALSE(fb.get(0, 0));
    TEST_ASSERT_TRUE(fb.get(1, 0));
    TEST_ASSERT_TRUE(fb.get(2, 0));
    TEST_ASSERT_TRUE(fb.get(3, 0));
    TEST_ASSERT_FALSE(fb.get(4, 0));
}
void test_unknown_char_skipped(void) {
    FrameBuffer fb; fb.clear();
    fb_text(fb, 0, 0, "@", 1);        // '@' not in font -> nothing drawn, no crash
    TEST_ASSERT_FALSE(fb.get(1, 0));
}
void test_number_renders_digits(void) {
    FrameBuffer fb; fb.clear();
    fb_number(fb, 0, 0, 12, 1);       // draws "12"; second glyph starts at x=6
    TEST_ASSERT_TRUE(fb.get(1, 0) || fb.get(2, 0));   // '1' has pixels in top rows
    TEST_ASSERT_TRUE(fb.get(7, 0) || fb.get(8, 0));   // '2' second char present
}
void test_scale_enlarges(void) {
    FrameBuffer fb; fb.clear();
    fb_text(fb, 0, 0, "0", 2);        // scale 2 -> col1 becomes a 2x2 block at (2..3, 0..1)
    TEST_ASSERT_TRUE(fb.get(2, 0));
    TEST_ASSERT_TRUE(fb.get(3, 1));
}

void test_alphabet_glyphs_exist(void) {
    for (char c = 'A'; c <= 'Z'; ++c) TEST_ASSERT_NOT_NULL(font_glyph(c));
    for (char c = 'a'; c <= 'z'; ++c) TEST_ASSERT_NOT_NULL(font_glyph(c));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_glyph_zero_top_row);
    RUN_TEST(test_unknown_char_skipped);
    RUN_TEST(test_number_renders_digits);
    RUN_TEST(test_scale_enlarges);
    RUN_TEST(test_alphabet_glyphs_exist);
    return UNITY_END();
}
