#include "modules/touch_input.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace {
int16_t median(const int16_t *v, int n) {
    int16_t tmp[TouchTracker::MAX_SAMPLES];
    std::copy(v, v + n, tmp);
    std::nth_element(tmp, tmp + n / 2, tmp + n);
    return tmp[n / 2];
}

int scale_axis(int raw, int raw0, int raw1, int px0, int px1, int max_px) {
    if (raw1 == raw0) return px0;
    const int v = px0 + (raw - raw0) * (px1 - px0) / (raw1 - raw0);
    return std::max(0, std::min(max_px, v));
}
}

void TouchTracker::reset() {
    state_ = State::Idle;
    n_ = 0;
    lockout_until_ms_ = 0;
}

void TouchTracker::add(const TouchRawSample &s) {
    if (skipped_ < SKIP_EDGE_SAMPLES) { ++skipped_; return; }
    if (n_ < MAX_SAMPLES) { xs_[n_] = s.raw_x; ys_[n_] = s.raw_y; ++n_; }
}

bool TouchTracker::current(int16_t &raw_x, int16_t &raw_y) const {
    if (state_ != State::Down || n_ == 0) return false;
    raw_x = median(xs_, n_);
    raw_y = median(ys_, n_);
    return true;
}

TouchTap TouchTracker::feed(const TouchRawSample &s, uint32_t now) {
    TouchTap tap;
    switch (state_) {
    case State::Idle:
        if (!s.pressed || (int32_t)(now - lockout_until_ms_) < 0) break;
        state_ = State::Confirming;
        confirm_count_ = 1;
        n_ = 0;
        skipped_ = 0;
        press_ms_ = now;
        add(s);
        break;
    case State::Confirming:
        if (!s.pressed) { state_ = State::Idle; break; }
        add(s);
        if (++confirm_count_ >= PRESS_CONFIRM_SAMPLES) state_ = State::Down;
        break;
    case State::Down:
        if (s.pressed) { add(s); break; }
        state_ = State::Releasing;
        release_count_ = 1;
        release_ms_ = now;
        break;
    case State::Releasing:
        if (s.pressed) { state_ = State::Down; add(s); break; }
        if (++release_count_ < RELEASE_SAMPLES || now - release_ms_ < RELEASE_MS) break;
        if (n_ > 0 && release_ms_ - press_ms_ <= MAX_TAP_MS) {
            tap.valid = true;
            tap.raw_x = median(xs_, n_);
            tap.raw_y = median(ys_, n_);
        }
        state_ = State::Idle;
        lockout_until_ms_ = now + LOCKOUT_MS;
        break;
    }
    return tap;
}

void touch_map(const TouchCalibration &cal, int16_t raw_x, int16_t raw_y, int &x, int &y) {
    const int a = cal.swap_xy ? raw_y : raw_x;
    const int b = cal.swap_xy ? raw_x : raw_y;
    x = scale_axis(a, cal.ax0, cal.ax1, TOUCH_CAL_LEFT, TOUCH_CAL_RIGHT, FB_W - 1);
    y = scale_axis(b, cal.ay0, cal.ay1, TOUCH_CAL_TOP, TOUCH_CAL_BOTTOM, FB_H - 1);
}

bool touch_calibration_plausible(const TouchCalibration &cal) {
    return std::abs(cal.ax1 - cal.ax0) >= 800 && std::abs(cal.ay1 - cal.ay0) >= 600;
}

void TouchCalibrator::start() {
    active_ = true;
    step_ = 0;
    check_error_ = 0;
}

void TouchCalibrator::target(int i, int &x, int &y) const {
    static const int TX[POINTS] = {TOUCH_CAL_LEFT, TOUCH_CAL_RIGHT, TOUCH_CAL_RIGHT, TOUCH_CAL_LEFT, FB_W / 2};
    static const int TY[POINTS] = {TOUCH_CAL_TOP, TOUCH_CAL_TOP, TOUCH_CAL_BOTTOM, TOUCH_CAL_BOTTOM, FB_H / 2};
    x = TX[i];
    y = TY[i];
}

TouchCalibrator::Result TouchCalibrator::tap(int16_t raw_x, int16_t raw_y) {
    if (!active_) return Result::Failed;
    rx_[step_] = raw_x;
    ry_[step_] = raw_y;
    ++step_;
    if (step_ == 4) {
        // Corners: 0=TL 1=TR 2=BR 3=BL. Whichever raw axis moves more from left
        // to right is the screen X axis.
        const int dx_on_rx = std::abs((rx_[1] + rx_[2]) - (rx_[0] + rx_[3]));
        const int dx_on_ry = std::abs((ry_[1] + ry_[2]) - (ry_[0] + ry_[3]));
        TouchCalibration c;
        c.swap_xy = dx_on_ry > dx_on_rx;
        const int16_t *a = c.swap_xy ? ry_ : rx_;
        const int16_t *b = c.swap_xy ? rx_ : ry_;
        c.ax0 = (a[0] + a[3]) / 2;
        c.ax1 = (a[1] + a[2]) / 2;
        c.ay0 = (b[0] + b[1]) / 2;
        c.ay1 = (b[2] + b[3]) / 2;
        if (!touch_calibration_plausible(c)) { active_ = false; return Result::Failed; }
        cal_ = c;
    } else if (step_ == POINTS) {
        int x, y, tx, ty;
        touch_map(cal_, raw_x, raw_y, x, y);
        target(POINTS - 1, tx, ty);
        check_error_ = std::max(std::abs(x - tx), std::abs(y - ty));
        active_ = false;
        return check_error_ <= MAX_CHECK_ERROR_PX ? Result::Done : Result::Failed;
    }
    return Result::InProgress;
}

void touch_marker_draw(FrameBuffer &fb, int x, int y) {
    fb_hline(fb, x - 12, y, 25, true);
    fb_vline(fb, x, y - 12, 25, true);
    fb_rect(fb, x - 4, y - 4, 9, 9, false, true);
}

void touch_calibration_draw(FrameBuffer &fb, const TouchCalibrator &c, const char *status) {
    fb_text(fb, 58, 70, "TOUCH CALIBRATION", 2);
    if (status) fb_text(fb, 58, 150, status, 1);
    if (!c.active()) return;
    char b[40];
    std::snprintf(b, sizeof(b), "TAP THE CROSS  %d/%d", c.step() + 1, TouchCalibrator::POINTS);
    fb_text(fb, 100, 100, b, 1);
    fb_text(fb, 88, 165, "HOME BUTTON = CANCEL", 1);
    int x, y;
    c.target(c.step(), x, y);
    touch_marker_draw(fb, x, y);
}
