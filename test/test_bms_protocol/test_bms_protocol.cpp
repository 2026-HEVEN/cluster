#include <unity.h>
#include "bms_protocol.h"

namespace {
void put_u16le(uint8_t *d, uint16_t value) {
    d[0] = (uint8_t)(value & 0xFF);
    d[1] = (uint8_t)(value >> 8);
}

void put_u32le(uint8_t *d, uint32_t value) {
    d[0] = (uint8_t)(value & 0xFF);
    d[1] = (uint8_t)((value >> 8) & 0xFF);
    d[2] = (uint8_t)((value >> 16) & 0xFF);
    d[3] = (uint8_t)(value >> 24);
}

void make_summary_frame(uint8_t frame[32]) {
    for (int i = 0; i < 32; ++i) frame[i] = 0;
    frame[0] = 0x3A;
    frame[1] = 0x16;
    frame[2] = 0x2A;
    put_u32le(frame + 4, (uint32_t)(int32_t)-1234); // -1.234 A
    put_u32le(frame + 8, 50000);                    // 50.000 V
    frame[12] = 31;
    put_u32le(frame + 16, 72345);
    frame[24] = 78;
    frame[25] = 96;
    put_u16le(frame + 26, 321);
    frame[30] = 0x0D;
    frame[31] = 0x0A;
}
}

void test_expected_frame_lengths(void) {
    TEST_ASSERT_EQUAL_INT(32, bms_expected_frame_len(0x2A));
    TEST_ASSERT_EQUAL_INT(36, bms_expected_frame_len(0x24));
    TEST_ASSERT_EQUAL_INT(0, bms_expected_frame_len(0x99));
}

void test_decodes_summary_example(void) {
    uint8_t frame[32];
    make_summary_frame(frame);

    BmsSummary summary;
    TEST_ASSERT_TRUE(bms_decode_summary_frame(frame, sizeof(frame), summary));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.000f, summary.pack_voltage_v);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.234f, summary.current_a);
    TEST_ASSERT_EQUAL_INT(31, summary.temp_c);
    TEST_ASSERT_EQUAL_UINT32(72345, summary.remaining_mah);
    TEST_ASSERT_EQUAL_UINT8(78, summary.soc_pct);
    TEST_ASSERT_EQUAL_UINT8(96, summary.soh_pct);
    TEST_ASSERT_EQUAL_UINT16(321, summary.cycles);
}

void test_clamps_soc_to_100(void) {
    uint8_t frame[32];
    make_summary_frame(frame);
    frame[24] = 250;

    BmsSummary summary;
    TEST_ASSERT_TRUE(bms_decode_summary_frame(frame, sizeof(frame), summary));
    TEST_ASSERT_EQUAL_UINT8(100, summary.soc_pct);
}

void test_rejects_wrong_tail(void) {
    uint8_t frame[32];
    make_summary_frame(frame);
    frame[31] = 0;

    BmsSummary summary;
    TEST_ASSERT_FALSE(bms_decode_summary_frame(frame, sizeof(frame), summary));
}

void test_rejects_wrong_id(void) {
    uint8_t frame[32];
    make_summary_frame(frame);
    frame[2] = 0x24;

    BmsSummary summary;
    TEST_ASSERT_FALSE(bms_decode_summary_frame(frame, sizeof(frame), summary));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_expected_frame_lengths);
    RUN_TEST(test_decodes_summary_example);
    RUN_TEST(test_clamps_soc_to_100);
    RUN_TEST(test_rejects_wrong_tail);
    RUN_TEST(test_rejects_wrong_id);
    return UNITY_END();
}
