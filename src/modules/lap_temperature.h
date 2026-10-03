#pragma once
#include <cstdint>

constexpr uint8_t LAP_TEMPERATURE_MAX = 51;
constexpr uint32_t LAP_TEMPERATURE_FRESH_MS = 300;
constexpr int16_t LAP_TEMPERATURE_INVALID = -32768;

struct LapTemperatureSample {
    int16_t celsius = 0;
    uint32_t received_ms = 0;
    bool seen = false;
    constexpr LapTemperatureSample(int16_t value = 0, uint32_t received = 0,
                                   bool available = false)
        : celsius(value), received_ms(received), seen(available) {}
};

struct LapTemperatureSummary {
    int16_t mean_x10[2] = {LAP_TEMPERATURE_INVALID, LAP_TEMPERATURE_INVALID};
    int16_t rise_x10[2] = {LAP_TEMPERATURE_INVALID, LAP_TEMPERATURE_INVALID};
    bool mean_valid[2]{};
    bool rise_valid[2]{};
};

// Time-weighted controller temperatures for timer-running time only.
class LapTemperatureHistory {
public:
    void reset();
    void begin(uint8_t lap, uint32_t now, const LapTemperatureSample samples[2]);
    void update(uint32_t now, bool running, const LapTemperatureSample samples[2]);
    void complete(uint32_t now, const LapTemperatureSample samples[2]);
    LapTemperatureSummary summary(uint8_t lap) const;
private:
    LapTemperatureSummary completed_[LAP_TEMPERATURE_MAX]{};
    LapTemperatureSample previous_[2]{};
    int64_t integral_[2]{};
    uint64_t covered_ms_[2]{};
    uint64_t running_ms_ = 0;
    uint32_t last_ms_ = 0;
    uint8_t lap_ = 0;
    bool running_ = false;
    LapTemperatureSummary current() const;
};
