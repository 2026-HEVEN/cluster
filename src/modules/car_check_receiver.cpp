#include "car_check_receiver.h"
namespace {
uint16_t u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1])<<8; }
int32_t s16(const uint8_t *p) { const auto n=u16(p); return n<32768?n:int32_t(n)-65536; }
}
namespace car_check {
Steering decode_steering(const uint8_t *d) {
    Steering s; s.unit=s16(d)/1000.0f; s.validity_present=d[6]&128;
    s.valid=(d[6]&1) && s.unit>=-1 && s.unit<=1; s.life=d[7]; return s;
}
Imu decode_imu(const uint8_t *d) {
    Imu s; s.yaw_dps=s16(d)/100.0f; s.ax_g=s16(d+2)/100.0f; s.ay_g=s16(d+4)/100.0f;
    s.validity_present=d[6]&128; s.yaw_valid=d[6]&1; s.accel_valid=d[6]&2; s.life=d[7]; return s;
}
Wheels decode_wheels(const uint8_t *d) {
    Wheels s; for(int i=0;i<4;++i) { const auto raw=u16(d+2*i); s.valid[i]=raw!=INVALID_WHEEL; s.kph[i]=raw/10.0f; } return s;
}
Control decode_control(const uint8_t *d) {
    Control s; s.supported=d[0]==VERSION; s.tv_requested=d[1]&1; s.regen_requested=d[1]&2;
    s.paddock_requested=d[1]&4; s.tv_active=d[2]&1; s.regen_available=d[2]&2;
    s.regen_active=d[2]&4; s.paddock_active=d[2]&8; s.output_allowed=d[2]&16;
    s.cluster_fresh=d[2]&32; s.brake_installed=d[2]&64;
    s.tv_block=d[3]; s.regen_block=d[4]; s.life=d[7]; return s;
}
}
bool CarCheckReceiver::receive(uint32_t id,const uint8_t *d,size_t len,bool extended,bool rtr,uint32_t now) {
    if(!d || len!=8 || !extended || rtr) return false;
    using namespace car_check;
    switch(id) {
    case STEERING_ID: steering=decode_steering(d); steering_rx.received(now); return true;
    case IMU_ID: imu=decode_imu(d); imu_rx.received(now); return true;
    case WHEELS_ID: wheels=decode_wheels(d); wheels_rx.received(now); return true;
    case CONTROL_ID: control=decode_control(d); control_rx.received(now); return true;
    default: return false;
    }
}
