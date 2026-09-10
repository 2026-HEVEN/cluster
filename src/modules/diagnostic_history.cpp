#include "diagnostic_history.h"
#include <cmath>

namespace { void increment(uint32_t &v) { if(v!=UINT32_MAX) ++v; } }
void TelemetryValues::set(Channel ch,float v,bool ok) {
    const unsigned i=static_cast<unsigned>(ch);
    if(ok && std::isfinite(v)) { value[i]=v; valid|=1u<<i; }
    else { value[i]=0; valid&=~(1u<<i); }
}
void DiagnosticHistory::observe(uint32_t now,const TelemetryValues &v) {
    broken_|=static_cast<uint16_t>(~v.valid);
    while(count_ && now-at(0).ms>60000) { head_=(head_+1)%SAMPLE_CAPACITY; --count_; }
    if(!sampled_ || now-last_sample_>=500) {
        if(count_==SAMPLE_CAPACITY) { head_=(head_+1)%SAMPLE_CAPACITY; --count_; }
        auto &s=samples_[(head_+count_++)%SAMPLE_CAPACITY];
        s.ms=now; s.valid=v.valid; s.broken=broken_;
        for(size_t i=0;i<CHANNELS;++i) s.value[i]=v.value[i];
        broken_=0; sampled_=true; last_sample_=now;
    }
    // Keep the transition preceding the left edge to describe constant states.
    while(rtk_count_>1 && now-rtk_at(1).ms>=60000) { rtk_head_=(rtk_head_+1)%RTK_CAPACITY; --rtk_count_; }
    if(rtk_count_ && now-rtk_at(0).ms>60000) rtk_[rtk_head_].ms=now-60000;
    if(!rtk_count_ || rtk_at(rtk_count_-1).state!=v.rtk) {
        if(rtk_count_==RTK_CAPACITY) { rtk_head_=(rtk_head_+1)%RTK_CAPACITY; --rtk_count_; increment(rtk_overwritten_); }
        rtk_[(rtk_head_+rtk_count_++)%RTK_CAPACITY]={now,v.rtk};
    }
}
void DiagnosticHistory::append(LinkId id,EventKind kind,uint8_t bit,uint32_t now) {
    events_[event_head_]={now,id,kind,bit}; event_head_=(event_head_+1)%EVENT_CAPACITY;
    if(event_count_<EVENT_CAPACITY) ++event_count_; else increment(overwritten_);
}
void DiagnosticHistory::connection(LinkId id,bool live,uint32_t now) {
    auto &s=links_[static_cast<size_t>(id)];
    if(s.live==live) return;
    if(live) {
        append(id,s.seen?EventKind::Restored:EventKind::Connected,0,now);
        s.seen=true; s.since=now;
    } else { increment(s.drops); append(id,EventKind::Lost,0,now); }
    s.live=live;
}
void DiagnosticHistory::fault(LinkId id,uint32_t bitmap,bool valid,uint32_t now) {
    if(!valid) return; // A missing frame is not evidence that a fault cleared.
    const size_t i=static_cast<size_t>(id);
    const uint32_t changed=faults_[i]^bitmap;
    for(uint8_t bit=0;bit<24;++bit) if(changed&(1u<<bit)) {
        const bool on=(bitmap&(1u<<bit))!=0;
        if(on) increment(links_[i].faults);
        append(id,on?EventKind::FaultOn:EventKind::FaultOff,bit,now);
    }
    faults_[i]=bitmap;
}
const char *diagnostic_link_name(LinkId id) {
    static const char *names[]={"MOTOR L","MOTOR R","VCU","WSS","STEER","IMU","WHEELS","CONTROL","BMS","EM","GNSS","WIFI","NTRIP","RTCM"};
    return names[static_cast<size_t>(id)];
}
const char *diagnostic_rtk_name(uint8_t state) {
    static const char *names[]={"NO DATA","NO FIX","GNSS","DGPS","FLOAT","FIXED","OTHER"};
    return names[state<7?state:6];
}
const char *diagnostic_fault_name(uint8_t bit) {
    static const char *names[]={"OVER CURRENT","OVER LOAD","OVER VOLT","LOW VOLT","CTRL HOT","MOTOR HOT","STALL","MOTOR PHASE","MOTOR SENSOR","AUX SENSOR","ENCODER ALIGN","RUNAWAY","MAIN ACCEL","AUX ACCEL","PRECHARGE","DC CONTACTOR","POWER VALVE","CURRENT SENSOR","AUTO TUNE","RS485","CAN","SOFTWARE","BIT22","BIT23"};
    return names[bit<24?bit:23];
}
