#include <unity.h>
#include "modules/drivetrain_monitor.h"

void setUp() {}
void tearDown() {}

namespace {
void pair(DrivetrainMonitor &m, uint32_t now, int left, int right) {
    m.receive(DrivetrainSide::Left, left, now);
    m.receive(DrivetrainSide::Right, right, now);
}

DrivetrainMonitorEvent dropout(DrivetrainMonitor &m, DrivetrainSide side,
                               uint32_t now, int moving = 500) {
    const DrivetrainSide other = side == DrivetrainSide::Left
        ? DrivetrainSide::Right : DrivetrainSide::Left;
    m.receive(other, -moving, now);
    m.receive(side, moving, now);
    m.receive(side, 0, now + 50);
    m.receive(other, -moving, now + 75);
    return m.receive(side, moving, now + 100);
}
}

void test_stationary_zero_does_not_trigger() {
    DrivetrainMonitor m;
    for (uint32_t t = 0; t <= 3000; t += 50) pair(m, t, 0, 0);
    TEST_ASSERT_TRUE(m.status().left == DrivetrainFault::None);
    TEST_ASSERT_TRUE(m.status().right == DrivetrainFault::None);
    TEST_ASSERT_EQUAL_UINT16(0, m.status().left_dropouts);
}

void test_dropout_requires_recovery_and_three_events() {
    DrivetrainMonitor m;
    TEST_ASSERT_FALSE(dropout(m, DrivetrainSide::Right, 100).triggered);
    TEST_ASSERT_FALSE(dropout(m, DrivetrainSide::Right, 700).triggered);
    const auto event = dropout(m, DrivetrainSide::Right, 1300);
    TEST_ASSERT_TRUE(event.triggered);
    TEST_ASSERT_TRUE(event.side == DrivetrainSide::Right);
    TEST_ASSERT_TRUE(event.fault == DrivetrainFault::RpmDropout);
    TEST_ASSERT_EQUAL_UINT16(3, m.status().right_dropouts);
}

void test_dropouts_outside_window_do_not_trigger() {
    DrivetrainMonitor m;
    dropout(m, DrivetrainSide::Left, 100);
    dropout(m, DrivetrainSide::Left, 1500);
    const auto event = dropout(m, DrivetrainSide::Left, 2500);
    TEST_ASSERT_FALSE(event.triggered);
    TEST_ASSERT_TRUE(m.status().left == DrivetrainFault::None);
}

void test_zero_run_counts_once_and_times_out() {
    DrivetrainMonitor m;
    pair(m, 100, 500, -500);
    m.receive(DrivetrainSide::Left, 0, 150);
    m.receive(DrivetrainSide::Left, 0, 200);
    m.receive(DrivetrainSide::Right, -500, 250);
    m.receive(DrivetrainSide::Left, 500, 350);
    TEST_ASSERT_EQUAL_UINT16(0, m.status().left_dropouts);
}

void test_stale_opposite_does_not_confirm_dropout() {
    DrivetrainMonitor m;
    pair(m, 100, 500, -500);
    m.receive(DrivetrainSide::Left, 0, 150);
    m.receive(DrivetrainSide::Left, 500, 300);
    TEST_ASSERT_EQUAL_UINT16(0, m.status().left_dropouts);
}

void test_stale_own_history_does_not_start_dropout() {
    DrivetrainMonitor m;
    pair(m, 100, 500, -500);
    m.receive(DrivetrainSide::Right, -500, 1000);
    m.receive(DrivetrainSide::Left, 0, 1000);
    m.receive(DrivetrainSide::Right, -500, 1050);
    m.receive(DrivetrainSide::Left, 500, 1050);
    TEST_ASSERT_EQUAL_UINT16(0, m.status().left_dropouts);
}

void test_divergence_requires_full_hold_time() {
    DrivetrainMonitor m;
    pair(m, 100, 200, -2000);
    pair(m, 150, 200, -2200);
    pair(m, 200, 200, -2200);
    pair(m, 250, 200, -2200);
    TEST_ASSERT_TRUE(m.status().right == DrivetrainFault::None);
    auto event = m.receive(DrivetrainSide::Left, 200, 299);
    TEST_ASSERT_FALSE(event.triggered);
    event = m.receive(DrivetrainSide::Right, -2200, 300);
    TEST_ASSERT_TRUE(event.triggered);
    TEST_ASSERT_TRUE(event.fault == DrivetrainFault::RpmDivergence);
    TEST_ASSERT_TRUE(event.side == DrivetrainSide::Right);
}

void test_divergence_interruption_resets_timer() {
    DrivetrainMonitor m;
    pair(m, 100, 200, -2000);
    pair(m, 200, 900, -1000);
    pair(m, 300, 200, -2000);
    pair(m, 350, 200, -2000);
    pair(m, 400, 200, -2000);
    pair(m, 450, 200, -2000);
    TEST_ASSERT_TRUE(m.status().right == DrivetrainFault::None);
    const auto event = m.receive(DrivetrainSide::Right, -2000, 500);
    TEST_ASSERT_TRUE(event.triggered);
}

void test_reverse_signs_use_magnitude() {
    DrivetrainMonitor m;
    pair(m, 100, -500, 500);
    m.receive(DrivetrainSide::Left, 0, 150);
    m.receive(DrivetrainSide::Right, 500, 175);
    m.receive(DrivetrainSide::Left, -500, 200);
    TEST_ASSERT_EQUAL_UINT16(1, m.status().left_dropouts);
}

void test_timestamp_wraparound() {
    DrivetrainMonitor m;
    const uint32_t start = UINT32_MAX - 149;
    pair(m, start, 200, -2000);
    pair(m, UINT32_MAX - 99, 200, -2000);
    pair(m, UINT32_MAX - 49, 200, -2000);
    pair(m, 0, 200, -2000);
    const auto event = m.receive(DrivetrainSide::Left, 200, 50);
    TEST_ASSERT_TRUE(event.triggered);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_stationary_zero_does_not_trigger);
    RUN_TEST(test_dropout_requires_recovery_and_three_events);
    RUN_TEST(test_dropouts_outside_window_do_not_trigger);
    RUN_TEST(test_zero_run_counts_once_and_times_out);
    RUN_TEST(test_stale_opposite_does_not_confirm_dropout);
    RUN_TEST(test_stale_own_history_does_not_start_dropout);
    RUN_TEST(test_divergence_requires_full_hold_time);
    RUN_TEST(test_divergence_interruption_resets_timer);
    RUN_TEST(test_reverse_signs_use_magnitude);
    RUN_TEST(test_timestamp_wraparound);
    return UNITY_END();
}
