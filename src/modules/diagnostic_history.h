#pragma once
#include <cstdint>
#include <cstddef>

enum class Channel : uint8_t { Wss, BusL, BusR, PhaseL, PhaseR, VoltL, VoltR,
    EmHv, EmLv, BmsV, EmA, Throttle, Count };
constexpr size_t CHANNELS = static_cast<size_t>(Channel::Count);
enum class LinkId : uint8_t { MotorL, MotorR, Vcu, Wss, Steering, Imu, Wheels,
    Control, Bms, Em, Gps, Wifi, Ntrip, Rtcm, Count };
constexpr size_t LINKS = static_cast<size_t>(LinkId::Count);
enum class EventKind : uint8_t { Connected, Lost, Restored, FaultOn, FaultOff };
struct DiagnosticEvent { uint32_t ms; LinkId node; EventKind kind; uint8_t bit; };
struct ConnectionStats {
    bool seen=false, live=false;
    uint32_t since=0, drops=0, faults=0;
};
struct TelemetryValues {
    float value[CHANNELS]{};
    uint16_t valid=0;
    uint8_t rtk=0; // 0=no data, 1=no fix, 2=GNSS, 3=DGPS, 4=float, 5=fixed, 6=other
    void set(Channel ch, float v, bool ok);
    bool has(Channel ch) const { return (valid & (1u << static_cast<unsigned>(ch))) != 0; }
    float get(Channel ch) const { return value[static_cast<unsigned>(ch)]; }
};
struct TelemetrySample { uint32_t ms; float value[CHANNELS]; uint16_t valid, broken; };
struct RtkTransition { uint32_t ms; uint8_t state; };
class DiagnosticHistory {
public:
    static constexpr size_t SAMPLE_CAPACITY=121, EVENT_CAPACITY=32, RTK_CAPACITY=256;
    void observe(uint32_t now, const TelemetryValues &values);
    void connection(LinkId id, bool live, uint32_t now);
    void fault(LinkId id, uint32_t bitmap, bool valid, uint32_t now);
    const ConnectionStats &connection(LinkId id) const { return links_[static_cast<size_t>(id)]; }
    size_t size() const { return count_; }
    const TelemetrySample &at(size_t i) const { return samples_[(head_+i)%SAMPLE_CAPACITY]; }
    size_t event_count() const { return event_count_; }
    const DiagnosticEvent &event(size_t newest_index) const {
        return events_[(event_head_+EVENT_CAPACITY-1-newest_index)%EVENT_CAPACITY];
    }
    uint32_t event_overwrites() const { return overwritten_; }
    size_t rtk_count() const { return rtk_count_; }
    const RtkTransition &rtk_at(size_t i) const { return rtk_[(rtk_head_+i)%RTK_CAPACITY]; }
    uint32_t rtk_overwrites() const { return rtk_overwritten_; }
private:
    TelemetrySample samples_[SAMPLE_CAPACITY]{};
    ConnectionStats links_[LINKS]{};
    uint32_t faults_[LINKS]{};
    DiagnosticEvent events_[EVENT_CAPACITY]{};
    RtkTransition rtk_[RTK_CAPACITY]{};
    size_t head_=0,count_=0,event_head_=0,event_count_=0,rtk_head_=0,rtk_count_=0;
    uint16_t broken_=0;
    bool sampled_=false;
    uint32_t last_sample_=0,overwritten_=0,rtk_overwritten_=0;
    void append(LinkId id, EventKind kind, uint8_t bit, uint32_t now);
};
const char *diagnostic_link_name(LinkId id);
static_assert(sizeof(DiagnosticHistory)<=10*1024,"Diagnostic history must fit the fixed RAM budget");
const char *diagnostic_rtk_name(uint8_t state);
const char *diagnostic_fault_name(uint8_t bit);
