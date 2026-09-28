#include "drivetrain_monitor.h"

#include <climits>

namespace {
uint8_t index_of(DrivetrainSide side) {
    return side == DrivetrainSide::Left ? 0 : 1;
}

int magnitude(int value) {
    return value < 0 ? -value : value;
}

void increment(uint16_t &value) {
    if (value != UINT16_MAX) ++value;
}
}

DrivetrainMonitor::DrivetrainMonitor(const DrivetrainMonitorConfig &config)
    : config_(config) {
    if (config_.dropout_limit == 0) config_.dropout_limit = 1;
    if (config_.dropout_limit > SideState::MAX_DROPOUT_LIMIT) {
        config_.dropout_limit = SideState::MAX_DROPOUT_LIMIT;
    }
}

void DrivetrainMonitor::reset() {
    sides_[0] = SideState{};
    sides_[1] = SideState{};
    status_ = DrivetrainMonitorStatus{};
    divergence_pending_ = false;
    divergence_reported_ = false;
    divergence_side_ = DrivetrainSide::Left;
    divergence_started_ms_ = 0;
}

DrivetrainMonitorEvent DrivetrainMonitor::confirm_dropout(DrivetrainSide side,
                                                           uint32_t now) {
    const uint8_t index = index_of(side);
    SideState &current = sides_[index];
    uint16_t &total = index == 0 ? status_.left_dropouts : status_.right_dropouts;
    uint32_t &last = index == 0 ? status_.left_last_event_ms : status_.right_last_event_ms;
    DrivetrainFault &latched = index == 0 ? status_.left : status_.right;

    increment(total);
    last = now;
    current.dropout_times[current.dropout_head] = now;
    current.dropout_head = static_cast<uint8_t>(
        (current.dropout_head + 1) % config_.dropout_limit);
    if (current.dropout_count < config_.dropout_limit) ++current.dropout_count;

    if (current.dropout_count < config_.dropout_limit) {
        return {};
    }

    const uint32_t oldest = current.dropout_times[current.dropout_head];
    if (now - oldest > config_.dropout_window_ms || latched != DrivetrainFault::None) {
        return {};
    }

    latched = DrivetrainFault::RpmDropout;
    DrivetrainMonitorEvent event;
    event.triggered = true;
    event.side = side;
    event.fault = DrivetrainFault::RpmDropout;
    event.ms = now;
    return event;
}

DrivetrainMonitorEvent DrivetrainMonitor::evaluate_divergence(uint32_t now) {
    const SideState &left = sides_[0];
    const SideState &right = sides_[1];
    if (!left.seen || !right.seen ||
        now - left.rx_ms > config_.pair_fresh_ms ||
        now - right.rx_ms > config_.pair_fresh_ms) {
        divergence_pending_ = false;
        divergence_reported_ = false;
        return {};
    }

    const int left_rpm = magnitude(left.rpm);
    const int right_rpm = magnitude(right.rpm);
    const int high = left_rpm > right_rpm ? left_rpm : right_rpm;
    const int low = left_rpm > right_rpm ? right_rpm : left_rpm;
    const DrivetrainSide dominant = left_rpm > right_rpm
        ? DrivetrainSide::Left : DrivetrainSide::Right;
    const bool divergent = high > config_.divergence_rpm &&
        static_cast<int64_t>(high) >
            static_cast<int64_t>(config_.divergence_ratio) * low;

    if (!divergent) {
        divergence_pending_ = false;
        divergence_reported_ = false;
        return {};
    }
    if (!divergence_pending_ || dominant != divergence_side_) {
        divergence_pending_ = true;
        divergence_reported_ = false;
        divergence_side_ = dominant;
        divergence_started_ms_ = now;
        return {};
    }
    if (divergence_reported_ || now - divergence_started_ms_ < config_.divergence_hold_ms) {
        return {};
    }

    divergence_reported_ = true;
    const uint8_t index = index_of(dominant);
    DrivetrainFault &latched = index == 0 ? status_.left : status_.right;
    uint32_t &last = index == 0 ? status_.left_last_event_ms : status_.right_last_event_ms;
    last = now;
    if (latched == DrivetrainFault::RpmDivergence) return {};
    latched = DrivetrainFault::RpmDivergence;
    DrivetrainMonitorEvent event;
    event.triggered = true;
    event.side = dominant;
    event.fault = DrivetrainFault::RpmDivergence;
    event.ms = now;
    return event;
}

DrivetrainMonitorEvent DrivetrainMonitor::receive(DrivetrainSide side, int rpm,
                                                   uint32_t now) {
    const uint8_t index = index_of(side);
    const uint8_t other_index = index == 0 ? 1 : 0;
    SideState &current = sides_[index];
    const SideState &other = sides_[other_index];
    DrivetrainMonitorEvent event;

    if (current.zero_pending) {
        if (rpm != 0) {
            const bool recovered = magnitude(rpm) > config_.moving_rpm &&
                now - current.zero_started_ms <= config_.dropout_recovery_ms;
            const bool other_moving = other.seen &&
                now - other.rx_ms <= config_.pair_fresh_ms &&
                magnitude(other.rpm) > config_.moving_rpm;
            current.zero_pending = false;
            if (recovered && other_moving) event = confirm_dropout(side, now);
        } else if (now - current.zero_started_ms > config_.dropout_recovery_ms) {
            current.zero_pending = false;
        }
    } else if (rpm == 0 && current.seen &&
               now - current.rx_ms <= config_.pair_fresh_ms &&
               magnitude(current.rpm) > config_.moving_rpm && other.seen &&
               now - other.rx_ms <= config_.pair_fresh_ms &&
               magnitude(other.rpm) > config_.moving_rpm) {
        current.zero_pending = true;
        current.zero_started_ms = now;
    }

    current.rpm = rpm;
    current.rx_ms = now;
    current.seen = true;

    const DrivetrainMonitorEvent divergence = evaluate_divergence(now);
    return divergence.triggered ? divergence : event;
}
