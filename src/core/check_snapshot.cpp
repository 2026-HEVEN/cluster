#include "core/check_snapshot.h"
#include "state.h"
#include "core/ntrip.h"
#include "core/gps_laptimer.h"
#include <cstdio>
#include <cstdarg>
#include <cmath>

namespace {
bool fresh(uint32_t stamp,uint32_t now,uint32_t limit) { return stamp && now-stamp<=limit; }
const char *link(uint32_t stamp,uint32_t now,uint32_t limit) {
    return !stamp ? "WAIT" : fresh(stamp,now,limit) ? "LIVE" : "STALE";
}
const char *on(bool value) { return value ? "ON" : "OFF"; }
void row(char (&rows)[12][2][25], int i, const char *key, const char *format, ...) {
    std::snprintf(rows[i][0],25,"%s",key);
    va_list args; va_start(args,format); std::vsnprintf(rows[i][1],25,format,args); va_end(args);
}
void value(char *out,size_t len,bool valid,float number,const char *unit,int digits=1) {
    if(valid) std::snprintf(out,len,"%.*f %s",digits,number,unit);
    else std::snprintf(out,len,"-- WAIT");
}
void time_row(char (&rows)[12][2][25],int i,const char *key,uint32_t ms) {
    row(rows,i,key,"%lu:%02lu.%02lu",static_cast<unsigned long>(ms/60000),
        static_cast<unsigned long>(ms/1000%60),static_cast<unsigned long>(ms/10%100));
}
}
void check_snapshot(CheckSnapshot &d, uint32_t now) {
    d.warning_count = 0;
    const uint32_t fb1[]={state.controller_l_fb1_last_ms,state.controller_r_fb1_last_ms};
    const uint32_t fb2[]={state.controller_l_fb2_last_ms,state.controller_r_fb2_last_ms};
    const float volts[]={state.bus_voltage,state.bus_voltage_r};
    const float amps[]={state.bus_current,state.bus_current_r};
    const float phase[]={state.phase_current,state.phase_current_r};
    const float rpm[]={state.speed_rpm_l,state.speed_rpm_r};
    const int mt[]={state.motor_temp,state.motor_temp_r},ct[]={state.controller_temp,state.controller_temp_r};
    const uint8_t status[]={state.controller_status,state.controller_status_r};
    const uint8_t errors[2][3]={{state.error1,state.error2,state.error3},{state.error1_r,state.error2_r,state.error3_r}};
    const char *keys[]={"LINK I/II","BUS V","BUS A","PHASE A","RPM","MOTOR C","CTRL C","STATUS","FAULT"};
    for(int i=0;i<9;++i) std::snprintf(d.motor[i][0],20,"%s",keys[i]);
    for(int s=0;s<2;++s) {
        const bool a=fresh(fb1[s],now,300),b=fresh(fb2[s],now,300);
        std::snprintf(d.motor[0][s+1],20,"%s/%s",link(fb1[s],now,300),link(fb2[s],now,300));
        value(d.motor[1][s+1],20,a,volts[s],"V");
        value(d.motor[2][s+1],20,a,amps[s],"A");
        value(d.motor[3][s+1],20,a,phase[s],"A");
        value(d.motor[4][s+1],20,a,rpm[s],"",0);
        value(d.motor[5][s+1],20,b,mt[s],"C",0);
        value(d.motor[6][s+1],20,b,ct[s],"C",0);
        if(b) {
            std::snprintf(d.motor[7][s+1],20,"0x%02X",status[s]);
            std::snprintf(d.motor[8][s+1],20,"%s %02X/%02X/%02X",
                errors[s][0]||errors[s][1]||errors[s][2] ? "ERR":"OK",errors[s][0],errors[s][1],errors[s][2]);
        } else {
            std::snprintf(d.motor[7][s+1],20,"-- WAIT");
            std::snprintf(d.motor[8][s+1],20,"-- WAIT");
        }
    }
    const bool bms=state.bms_ble_connected&&fresh(state.bms_last_rx_ms,now,5000);
    const bool em=state.em_record_seen&&now-state.em_record_last_ms<=500;
    row(d.power,0,"BMS / EM","%s / %s",bms?"LIVE":"WAIT",em?"LIVE":"WAIT");
    const char *pkeys[]={"BMS SOC","BMS VOLTAGE","BMS CURRENT","BMS TEMP","EM HV VOLTAGE","EM LV VOLTAGE","EM HV CURRENT","EM CPU TEMP"};
    const float pvalues[]={state.soc*100,state.bms_pack_voltage,state.bms_current,static_cast<float>(state.bms_temp_c),state.em_hv_decivolts/10.0f,state.em_lv_centivolts/100.0f,state.em_current_deciamps/10.0f,state.em_cpu_centidegrees/100.0f};
    const char *units[]={"%","V","A","C","V","V","A","C"};
    for(int i=0;i<8;++i) {
        row(d.power,i+1,pkeys[i],"");
        value(d.power[i+1][1],25,i<4 ? bms&&(i!=0||state.soc_valid) : em,pvalues[i],units[i],i==5?2:1);
    }
    row(d.power,9,"BMS REMAIN / Ah",bms?"%.2f":"-- WAIT",state.bms_remaining_mah/1000.0);
    row(d.power,10,"BMS SOH / %",bms?"%u":"-- WAIT",state.bms_soh);
    row(d.power,11,"BMS CYCLES",bms?"%u":"-- WAIT",state.bms_cycles);
    const bool vcu=fresh(state.vcu_cluster_status_last_ms,now,300);
    const bool wss=state.wss_valid&&fresh(state.vehicle_speed_last_rx_ms,now,300);
    row(d.vcu,0,"VCU LINK","%s",link(state.vcu_cluster_status_last_ms,now,300));
    const char *gears[]={"N","R","D","P"};
    row(d.vcu,1,"GEAR / BRAKE",vcu&&state.gear_from_can?"%s / %s":"-- WAIT",gears[state.gear<=3?state.gear:0],on(state.brake));
    row(d.vcu,2,"HV ACTIVE",vcu?"%s":"-- WAIT",on(state.hv_active));
    row(d.vcu,3,"THROTTLE",state.throttle_valid&&fresh(state.throttle_last_rx_ms,now,300)?"%.0f %%":"-- WAIT",state.throttle_pct);
    row(d.vcu,4,"WSS REPRESENTATIVE",""); value(d.vcu[4][1],25,wss,state.wss_kph,"km/h");
    const auto &cc=state.car_check;
    const auto &c=cc.control;
    const auto cq=cc.control_rx.quality(now,true,c.supported);
    d.control_valid=cq==car_check::Quality::Valid;
    const char *cqtext=car_check::quality_label(cq);
    row(d.vcu,5,"TV REQ / ACTIVE >","%s / %s",on(state.tc_enabled),d.control_valid?on(c.tv_active):cqtext);
    row(d.vcu,6,"RGN REQ / ACTIVE >","%s / %s",on(state.regen_level>0),d.control_valid?on(c.regen_active):cqtext);
    row(d.vcu,7,"PDK REQ / APPLIED","%s / %s",on(state.paddock),vcu?on(state.paddock_active):"WAIT");
    const auto wq=cc.wheels_rx.quality(now,true);
    unsigned wheel_valid=0;for(bool valid:cc.wheels.valid) if(valid) ++wheel_valid;
    if(wq==car_check::Quality::Valid) row(d.vcu,8,"4 WHEELS >","%u/4 VALID",wheel_valid);
    else row(d.vcu,8,"4 WHEELS >","%s",car_check::quality_label(wq));
    row(d.vcu,9,"STEERING >","%s",car_check::quality_label(cc.steering_rx.quality(now,cc.steering.valid,cc.steering.validity_present)));
    row(d.vcu,10,"IMU >","%s",car_check::quality_label(cc.imu_rx.quality(now,cc.imu.yaw_valid&&cc.imu.accel_valid,cc.imu.validity_present)));
    row(d.vcu,11,"BRAKE PRESSURE","NOT PROVIDED");
    const char *wk[]={"WSS FL / km/h","WSS FR / km/h","WSS RL / km/h","WSS RR / km/h"};
    for(int i=0;i<4;++i) {
        const auto q=cc.wheels_rx.quality(now,cc.wheels.valid[i]);
        if(q==car_check::Quality::Valid) row(d.sensors,i,wk[i],"%.1f OK",cc.wheels.kph[i]);
        else row(d.sensors,i,wk[i],"-- %s",car_check::quality_label(q));
    }
    const car_check::Quality qualities[]={cc.steering_rx.quality(now,cc.steering.valid,cc.steering.validity_present),
        cc.imu_rx.quality(now,cc.imu.yaw_valid,cc.imu.validity_present),
        cc.imu_rx.quality(now,cc.imu.accel_valid,cc.imu.validity_present)};
    const float sv[]={cc.steering.unit,cc.imu.yaw_dps,cc.imu.ax_g,cc.imu.ay_g};
    const char *sk[]={"STEER / norm","YAW / deg/s","ACCEL X / g","ACCEL Y / g"};
    for(int i=0;i<4;++i) {
        const auto q=qualities[i<2?i:2];
        if(q==car_check::Quality::Valid) row(d.sensors,4+i,sk[i],"%.3f OK",sv[i]);
        else row(d.sensors,4+i,sk[i],"-- %s",car_check::quality_label(q));
    }
    row(d.sensors,8,"STEER LIFE",cc.steering_rx.seen?"%u %s":"-- WAIT",cc.steering.life,car_check::quality_label(cc.steering_rx.quality(now,true)));
    row(d.sensors,9,"IMU LIFE",cc.imu_rx.seen?"%u %s":"-- WAIT",cc.imu.life,car_check::quality_label(cc.imu_rx.quality(now,true)));
    row(d.sensors,10,"BRAKE PRESSURE","NOT PROVIDED");
    row(d.sensors,11,"AXES / SIGN","CAL PENDING");
    const char *ck[]={"TV LOCAL / VCU REQ","TV ACTIVE","RGN LOCAL / VCU REQ","RGN AVAILABLE","RGN ACTIVE",
        "PDK LOCAL / VCU REQ","PDK ACTIVE","OUTPUT ALLOWED","CLUSTER RX FRESH","BRAKE SENSOR","VERSION / LIFE","BLOCK REASONS >"};
    for(int i=0;i<12;++i) row(d.control,i,ck[i],"-- %s",cqtext);
    if(d.control_valid) {
        row(d.control,0,ck[0],"%s / %s",on(state.tc_enabled),on(c.tv_requested));
        row(d.control,1,ck[1],"%s",on(c.tv_active));
        row(d.control,2,ck[2],"%s / %s",on(state.regen_level>0),on(c.regen_requested));
        row(d.control,3,ck[3],"%s",on(c.regen_available));row(d.control,4,ck[4],"%s",on(c.regen_active));
        row(d.control,5,ck[5],"%s / %s",on(state.paddock),on(c.paddock_requested));
        row(d.control,6,ck[6],"%s",on(c.paddock_active));row(d.control,7,ck[7],"%s",on(c.output_allowed));
        row(d.control,8,ck[8],"%s",on(c.cluster_fresh));row(d.control,9,ck[9],"%s",on(c.brake_installed));
        row(d.control,10,ck[10],"v1 / %u",c.life);
        row(d.control,11,ck[11],"TV %02X / RGN %02X",c.tv_block,c.regen_block);
    }
    const char *tv[]={"REQUEST OFF","GAINS ZERO","IMU INVALID","SPEED INVALID","LOW SPEED","OUTPUT BLOCKED","TEST OVERRIDE"};
    const char *rg[]={"REQUEST OFF","NOT VALIDATED","NO BRAKE SENSOR","BMS INVALID","OUTPUT BLOCKED","NO BRAKE DEMAND","SOC BLOCKED","DIRECTION MISMATCH"};
    for(int i=0;i<15;++i) std::snprintf(d.reasons[i],40,"%s %s: %s",i<7?"TV":"RGN",i<7?tv[i]:rg[i-7],
        ((i<7?c.tv_block:c.regen_block)&(1u<<(i<7?i:i-7)))?"YES":"NO");
    const bool gps=fresh(state.gps_last_rx_ms,now,3000);
    const bool fix=gps&&state.gps_fix_ok;
    row(d.gps,0,"FIX / RTK","%s",fresh(gps_laptimer::last_gga_ms(),now,3000)?gps_laptimer::rtk_status_label():"-- WAIT");
    row(d.gps,1,"LATITUDE",fix?"%.7f":"-- WAIT",state.gps_latitude);
    row(d.gps,2,"LONGITUDE",fix?"%.7f":"-- WAIT",state.gps_longitude);
    row(d.gps,3,"SAT / HDOP",gps?"%u / %.1f":"-- WAIT",gps_laptimer::satellites(),gps_laptimer::hdop());
    row(d.gps,4,"WI-FI / NTRIP","%s / %s",on(ntrip::wifi_connected()),on(ntrip::connected()));
    const uint32_t rtcm=ntrip::last_rtcm_ms();
    row(d.gps,5,"RTCM RX AGE",rtcm?"%.1fs %s":"-- WAIT",(now-rtcm)/1000.0,rtcm&&now-rtcm<=5000?"FRESH":"OLD");
    row(d.gps,6,"NTRIP DETAIL","%s",ntrip::status_label());
    row(d.gps,7,"LAP / RUNNING","%u / %s",gps_laptimer::current_lap_number(),on(gps_laptimer::timer_running()));
    time_row(d.gps,8,"CURRENT LAP",state.current_lap_ms);
    time_row(d.gps,9,"LAST LAP",state.last_lap_ms);
    time_row(d.gps,10,"BEST LAP",state.best_lap_ms);
    row(d.gps,11,"PPS","%s",link(state.gps_pps_last_ms,now,2000));
    std::snprintf(d.summaries[0][0],24,"L %s",d.motor[0][1]);
    std::snprintf(d.summaries[0][1],24,"R %s",d.motor[0][2]);
    std::snprintf(d.summaries[1][0],24,"BMS %s",bms?"LIVE":"WAIT");
    std::snprintf(d.summaries[1][1],24,"EM %s",em?"LIVE":"WAIT");
    std::snprintf(d.summaries[2][0],24,"VCU %s",vcu?"LIVE":"WAIT");
    std::snprintf(d.summaries[2][1],24,"WSS %s",wss?"LIVE":"WAIT");
    std::snprintf(d.summaries[3][0],24,"%.23s",d.gps[0][1]);
    std::snprintf(d.summaries[3][1],24,"NTRIP %s",on(ntrip::connected()));
}

void check_observe(DiagnosticHistory &h,TelemetryValues &v,uint32_t now) {
    v=TelemetryValues{};
    const bool l=fresh(state.controller_l_fb1_last_ms,now,300),r=fresh(state.controller_r_fb1_last_ms,now,300);
    const bool l2=fresh(state.controller_l_fb2_last_ms,now,300),r2=fresh(state.controller_r_fb2_last_ms,now,300);
    const bool b=state.bms_ble_connected&&fresh(state.bms_last_rx_ms,now,5000);
    const bool em=state.em_record_seen&&now-state.em_record_last_ms<=500;
    const bool wss=state.wss_valid&&fresh(state.vehicle_speed_last_rx_ms,now,300);
    v.set(Channel::Wss,state.wss_kph,wss);
    v.set(Channel::BusL,state.bus_current,l);v.set(Channel::BusR,state.bus_current_r,r);
    v.set(Channel::PhaseL,state.phase_current,l);v.set(Channel::PhaseR,state.phase_current_r,r);
    v.set(Channel::VoltL,state.bus_voltage,l);v.set(Channel::VoltR,state.bus_voltage_r,r);
    v.set(Channel::EmHv,state.em_hv_decivolts/10.0f,em);v.set(Channel::EmLv,state.em_lv_centivolts/100.0f,em);
    v.set(Channel::EmA,state.em_current_deciamps/10.0f,em);v.set(Channel::BmsV,state.bms_pack_voltage,b);
    v.set(Channel::Throttle,state.throttle_pct,state.throttle_valid&&fresh(state.throttle_last_rx_ms,now,300));
    if(fresh(gps_laptimer::last_gga_ms(),now,3000)) {
        switch(gps_laptimer::fix_quality()) {
        case 0:v.rtk=1;break;case 1:v.rtk=2;break;case 2:v.rtk=3;break;
        case 5:v.rtk=4;break;case 4:v.rtk=5;break;default:v.rtk=6;break;
        }
    }
    h.connection(LinkId::MotorL,l&&l2,now);h.connection(LinkId::MotorR,r&&r2,now);
    h.connection(LinkId::Vcu,fresh(state.vcu_cluster_status_last_ms,now,300),now);
    h.connection(LinkId::Wss,fresh(state.vehicle_speed_last_rx_ms,now,300),now);
    const auto &c=state.car_check;
    const car_check::Reception *rx[]={&c.steering_rx,&c.imu_rx,&c.wheels_rx,&c.control_rx};
    const LinkId ids[]={LinkId::Steering,LinkId::Imu,LinkId::Wheels,LinkId::Control};
    for(int i=0;i<4;++i) h.connection(ids[i],rx[i]->seen&&now-rx[i]->last_ms<=300,now);
    h.connection(LinkId::Bms,b,now);h.connection(LinkId::Em,em,now);
    h.connection(LinkId::Gps,fresh(state.gps_last_rx_ms,now,3000),now);
    h.connection(LinkId::Wifi,ntrip::wifi_connected(),now);h.connection(LinkId::Ntrip,ntrip::connected(),now);
    h.connection(LinkId::Rtcm,fresh(ntrip::last_rtcm_ms(),now,5000),now);
    h.fault(LinkId::MotorL,uint32_t(state.error1)|(uint32_t(state.error2)<<8)|(uint32_t(state.error3)<<16),l2,now);
    h.fault(LinkId::MotorR,uint32_t(state.error1_r)|(uint32_t(state.error2_r)<<8)|(uint32_t(state.error3_r)<<16),r2,now);
    h.observe(now,v);
}
HomeData check_home_snapshot(uint32_t now) {
    HomeData d;
    d.speed=state.wss_kph;d.speed_ok=state.wss_valid&&fresh(state.vehicle_speed_last_rx_ms,now,300);
    d.throttle=state.throttle_pct;d.throttle_ok=state.throttle_valid&&fresh(state.throttle_last_rx_ms,now,300);
    d.gear=state.gear;d.gear_ok=state.gear_from_can&&fresh(state.vcu_cluster_status_last_ms,now,300);
    if(state.soc_valid&&state.bms_ble_connected&&fresh(state.bms_last_rx_ms,now,5000)) d.soc=std::lround(state.soc*100);
    d.em_ok=state.em_record_seen&&now-state.em_record_last_ms<=500;
    d.hv=state.em_hv_decivolts/10.0f;d.lv=state.em_lv_centivolts/100.0f;
    const float em_current=state.em_current_deciamps/10.0f;
    d.power_kw=std::fabs(d.hv*em_current)/1000;d.charging=em_current<0;
    d.lap=gps_laptimer::current_lap_number();d.best_lap=state.best_lap_count;d.lap_ms=state.current_lap_ms;d.best_ms=state.best_lap_ms;
    return d;
}
