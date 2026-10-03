#include <unity.h>
#include "modules/lap_temperature.h"

void setUp() {}
void tearDown() {}

void test_weighted_means_and_previous_lap_delta() {
    LapTemperatureHistory history;
    LapTemperatureSample samples[2] = {{40, 100, true}, {60, 100, true}};
    history.begin(1, 100, samples);
    samples[0] = {60, 200, true};
    history.update(200, true, samples);
    samples[0].received_ms = samples[1].received_ms = 400;
    history.complete(400, samples);
    auto first = history.summary(1);
    TEST_ASSERT_EQUAL_INT16(533, first.mean_x10[0]);
    TEST_ASSERT_EQUAL_INT16(600, first.mean_x10[1]);
    TEST_ASSERT_FALSE(first.rise_valid[0]);
    samples[0] = {50, 400, true};
    samples[1] = {65, 400, true};
    history.begin(2, 400, samples);
    history.complete(500, samples);
    auto second = history.summary(2);
    TEST_ASSERT_EQUAL_INT16(-33, second.rise_x10[0]);
    TEST_ASSERT_EQUAL_INT16(50, second.rise_x10[1]);
    TEST_ASSERT_TRUE(second.rise_valid[0]);
    TEST_ASSERT_EQUAL_INT16(533, history.summary(1).mean_x10[0]);
}

void test_pause_is_excluded() {
    LapTemperatureHistory history;
    LapTemperatureSample samples[2] = {{40, 100, true}, {40, 100, true}};
    history.begin(1, 100, samples);
    history.update(200, false, samples);
    samples[0] = samples[1] = {80, 10000, true};
    history.update(10000, true, samples);
    history.complete(10100, samples);
    TEST_ASSERT_EQUAL_INT16(600, history.summary(1).mean_x10[0]);
}

void test_stale_and_missing_channels_are_invalid() {
    LapTemperatureHistory history;
    LapTemperatureSample samples[2] = {{40, 100, true}, {0, 0, false}};
    history.begin(1, 100, samples);
    history.complete(1100, samples);
    TEST_ASSERT_FALSE(history.summary(1).mean_valid[0]);
    TEST_ASSERT_FALSE(history.summary(1).mean_valid[1]);
    TEST_ASSERT_EQUAL_INT16(LAP_TEMPERATURE_INVALID, history.summary(1).mean_x10[0]);
    samples[0] = {50, 1100, true};
    history.begin(2, 1100, samples);
    history.complete(1200, samples);
    TEST_ASSERT_TRUE(history.summary(2).mean_valid[0]);
    TEST_ASSERT_FALSE(history.summary(2).rise_valid[0]);
}

void test_coverage_threshold_and_wraparound() {
    LapTemperatureHistory history;
    const uint32_t start = 0xFFFFFF00u;
    LapTemperatureSample samples[2] = {{-10, start, true}, {60, start, true}};
    history.begin(51, start, samples);
    history.complete(start + 333, samples); // 300/333 >= 90% valid coverage
    TEST_ASSERT_TRUE(history.summary(51).mean_valid[0]);
    TEST_ASSERT_EQUAL_INT16(-100, history.summary(51).mean_x10[0]);
    TEST_ASSERT_FALSE(history.summary(52).mean_valid[0]);
    history.reset();
    TEST_ASSERT_FALSE(history.summary(51).mean_valid[0]);
    history.begin(1, 100, samples);
    samples[0] = samples[1] = {40, 100, true};
    history.begin(1, 100, samples);
    history.complete(434, samples); // 300/334 < 90% invalid
    TEST_ASSERT_FALSE(history.summary(1).mean_valid[0]);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_weighted_means_and_previous_lap_delta);
    RUN_TEST(test_pause_is_excluded);
    RUN_TEST(test_stale_and_missing_channels_are_invalid);
    RUN_TEST(test_coverage_threshold_and_wraparound);
    return UNITY_END();
}
