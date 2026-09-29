#pragma once
#include <cstdint>
#include "framebuffer.h"

// XPT2046 raw samples -> debounced taps in screen coordinates.
// Pure logic: the core feeds one sample per poll and owns SPI/NVS.

struct TouchRawSample {
    bool pressed;     // controller pressure above threshold
    int16_t raw_x;    // controller X axis (0..4095)
    int16_t raw_y;    // controller Y axis (0..4095)
};

struct TouchTap {
    bool valid = false;
    int16_t raw_x = 0, raw_y = 0;   // median of the press, controller axes
};

// Press/release debounce. A tap is reported once per physical press, as soon as
// the press is confirmed.
class TouchTracker {
public:
    static constexpr int MAX_SAMPLES = 32;
    static constexpr int PRESS_CONFIRM_SAMPLES = 2;   // drop single noisy spikes
    static constexpr int SKIP_EDGE_SAMPLES = 1;       // first contact sample is least accurate
    static constexpr uint32_t RELEASE_MS = 60;        // pressure must stay low this long
    static constexpr int RELEASE_SAMPLES = 2;
    static constexpr uint32_t LOCKOUT_MS = 120;       // after a release, before the next press

    TouchTap feed(const TouchRawSample &s, uint32_t now);
    bool pressed() const { return state_ == State::Down; }
    // Current press position (median so far); valid only while pressed().
    bool current(int16_t &raw_x, int16_t &raw_y) const;
    void reset();

private:
    enum class State { Idle, Confirming, Down, Releasing };
    State state_ = State::Idle;
    int confirm_count_ = 0;
    int release_count_ = 0;
    int skipped_ = 0;
    uint32_t release_ms_ = 0;
    uint32_t lockout_until_ms_ = 0;
    int16_t xs_[MAX_SAMPLES]{}, ys_[MAX_SAMPLES]{};
    int n_ = 0;
    void add(const TouchRawSample &s);
};

struct TouchCalibration {
    // Screen x = x0 + (raw_a - a0) * (x1 - x0) / (a1 - a0), where raw_a is the
    // controller axis that moves with screen X (raw_y when swap_xy).
    bool swap_xy = false;
    int16_t ax0 = 3622, ax1 = 478;    // raw at CAL_LEFT / CAL_RIGHT
    int16_t ay0 = 572, ay1 = 3528;    // raw at CAL_TOP / CAL_BOTTOM
};

constexpr int TOUCH_CAL_LEFT = 24, TOUCH_CAL_RIGHT = 295;
constexpr int TOUCH_CAL_TOP = 24, TOUCH_CAL_BOTTOM = 215;

void touch_map(const TouchCalibration &cal, int16_t raw_x, int16_t raw_y, int &x, int &y);
bool touch_calibration_plausible(const TouchCalibration &cal);

// Four corner targets + one centre check target.
class TouchCalibrator {
public:
    static constexpr int POINTS = 5;
    static constexpr int MAX_CHECK_ERROR_PX = 15;
    enum class Result { InProgress, Done, Failed };

    void start();
    bool active() const { return active_; }
    int step() const { return step_; }
    void target(int i, int &x, int &y) const;
    Result tap(int16_t raw_x, int16_t raw_y);   // feed a tap from TouchTracker
    const TouchCalibration &result() const { return cal_; }
    int check_error_px() const { return check_error_; }
    void cancel() { active_ = false; }

private:
    bool active_ = false;
    int step_ = 0;
    int16_t rx_[POINTS]{}, ry_[POINTS]{};
    TouchCalibration cal_;
    int check_error_ = 0;
};

void touch_calibration_draw(FrameBuffer &fb, const TouchCalibrator &c, const char *status);
void touch_marker_draw(FrameBuffer &fb, int x, int y);
