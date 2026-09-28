#pragma once

#include <cstdint>

enum class DrivetrainSide : uint8_t { Left = 0, Right = 1 };
enum class DrivetrainFault : uint8_t { None = 0, RpmDropout = 1, RpmDivergence = 2 };

struct DrivetrainMonitorConfig {
    int moving_rpm = 200;
    int divergence_rpm = 800;
    uint8_t divergence_ratio = 3;
    uint8_t dropout_limit = 3;
    uint32_t dropout_window_ms = 2000;
    uint32_t dropout_recovery_ms = 150;
    uint32_t pair_fresh_ms = 100;
    uint32_t divergence_hold_ms = 200;
};

struct DrivetrainMonitorEvent {
    bool triggered = false;
    DrivetrainSide side = DrivetrainSide::Left;
    DrivetrainFault fault = DrivetrainFault::None;
    uint32_t ms = 0;
};

struct DrivetrainMonitorStatus {
    DrivetrainFault left = DrivetrainFault::None;
    DrivetrainFault right = DrivetrainFault::None;
    uint16_t left_dropouts = 0;
    uint16_t right_dropouts = 0;
    uint32_t left_last_event_ms = 0;
    uint32_t right_last_event_ms = 0;
};

class DrivetrainMonitor {
public:
    explicit DrivetrainMonitor(const DrivetrainMonitorConfig &config = {});
    DrivetrainMonitorEvent receive(DrivetrainSide side, int rpm, uint32_t now);
    const DrivetrainMonitorStatus &status() const { return status_; }
    void reset();

private:
    struct SideState {
        int rpm = 0;
        uint32_t rx_ms = 0;
        bool seen = false;
        bool zero_pending = false;
        uint32_t zero_started_ms = 0;
        static constexpr uint8_t MAX_DROPOUT_LIMIT = 8;
        uint32_t dropout_times[MAX_DROPOUT_LIMIT]{};
        uint8_t dropout_head = 0;
        uint8_t dropout_count = 0;
    };

    DrivetrainMonitorConfig config_;
    SideState sides_[2]{};
    DrivetrainMonitorStatus status_{};
    bool divergence_pending_ = false;
    bool divergence_reported_ = false;
    DrivetrainSide divergence_side_ = DrivetrainSide::Left;
    uint32_t divergence_started_ms_ = 0;

    DrivetrainMonitorEvent confirm_dropout(DrivetrainSide side, uint32_t now);
    DrivetrainMonitorEvent evaluate_divergence(uint32_t now);
};
