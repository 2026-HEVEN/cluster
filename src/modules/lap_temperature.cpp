#include "lap_temperature.h"

void LapTemperatureHistory::reset() { *this = LapTemperatureHistory{}; }

void LapTemperatureHistory::begin(uint8_t lap, uint32_t now,
                                  const LapTemperatureSample samples[2]) {
    lap_ = lap <= LAP_TEMPERATURE_MAX ? lap : 0;
    last_ms_ = now;
    running_ = lap_ != 0;
    running_ms_ = 0;
    for (unsigned side = 0; side < 2; ++side) {
        integral_[side] = 0;
        covered_ms_[side] = 0;
        previous_[side] = samples[side];
    }
}

void LapTemperatureHistory::update(uint32_t now, bool running,
                                   const LapTemperatureSample samples[2]) {
    const uint32_t elapsed = now - last_ms_;
    if (lap_ != 0 && running_) {
        running_ms_ += elapsed;
        for (unsigned side = 0; side < 2; ++side) {
            const auto &sample = previous_[side];
            const uint32_t age = last_ms_ - sample.received_ms;
            if (!sample.seen || age >= LAP_TEMPERATURE_FRESH_MS) continue;
            const uint32_t remaining = LAP_TEMPERATURE_FRESH_MS - age;
            const uint32_t covered = elapsed < remaining ? elapsed : remaining;
            integral_[side] += static_cast<int64_t>(sample.celsius) * covered;
            covered_ms_[side] += covered;
        }
    }
    last_ms_ = now;
    running_ = running && lap_ != 0;
    for (unsigned side = 0; side < 2; ++side) previous_[side] = samples[side];
}

LapTemperatureSummary LapTemperatureHistory::current() const {
    LapTemperatureSummary result;
    if (lap_ == 0 || running_ms_ == 0) return result;
    for (unsigned side = 0; side < 2; ++side) {
        // Do not present a short fragment of a disconnected lap as its average.
        if (covered_ms_[side] == 0 || covered_ms_[side] * 10 < running_ms_ * 9) continue;
        const int64_t numerator = integral_[side] * 10;
        const int64_t half = static_cast<int64_t>(covered_ms_[side] / 2);
        result.mean_x10[side] = static_cast<int16_t>(
            (numerator + (numerator < 0 ? -half : half)) /
            static_cast<int64_t>(covered_ms_[side]));
        result.mean_valid[side] = true;
        if (lap_ > 1 && completed_[lap_ - 2].mean_valid[side]) {
            result.rise_x10[side] = result.mean_x10[side] - completed_[lap_ - 2].mean_x10[side];
            result.rise_valid[side] = true;
        }
    }
    return result;
}

void LapTemperatureHistory::complete(uint32_t now, const LapTemperatureSample samples[2]) {
    update(now, false, samples);
    if (lap_ != 0) completed_[lap_ - 1] = current();
    lap_ = 0;
}

LapTemperatureSummary LapTemperatureHistory::summary(uint8_t lap) const {
    if (lap == 0 || lap > LAP_TEMPERATURE_MAX) return LapTemperatureSummary{};
    return lap == lap_ ? current() : completed_[lap - 1];
}
