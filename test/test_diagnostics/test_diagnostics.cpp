#include <unity.h>
#include "modules/diagnostics.h"
#include "state.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <limits>
#include <filesystem>

ClusterState state;
static uint32_t mock_gga=599900,mock_rtcm=599900;
static uint8_t mock_quality=4;
namespace ntrip {
bool wifi_connected(){return true;}
bool connected(){return true;}
const char *status_label(){return "NTRIP OK";}
uint32_t last_rtcm_ms(){return mock_rtcm;}
}
namespace gps_laptimer {
bool timer_running(){return true;}
uint8_t satellites(){return 18;}
float hdop(){return .7f;}
const char *rtk_status_label(){return "RTK FIXED";}
uint32_t last_gga_ms(){return mock_gga;}
uint8_t fix_quality(){return mock_quality;}
uint8_t current_lap_number(){return 1;}
}
// Production adapter, with host substitutes only for the hardware getters.
#include "../../src/core/check_snapshot.cpp"
void setUp(){state=ClusterState{};mock_gga=mock_rtcm=599900;mock_quality=4;}
void tearDown(){}

void test_history_rate_and_retention(){
    DiagnosticHistory h;TelemetryValues v;v.set(Channel::Wss,25,true);
    for(uint32_t t=0;t<=70000;t+=20) h.observe(t,v);
    TEST_ASSERT_EQUAL(121,h.size());TEST_ASSERT_EQUAL(10000,h.at(0).ms);
    TEST_ASSERT_EQUAL(70000,h.at(120).ms);
    TEST_ASSERT_LESS_THAN(11000,sizeof(h));
}
void test_history_wrap_and_no_backfill(){
    DiagnosticHistory h;TelemetryValues v;v.set(Channel::Wss,0,true);
    h.observe(UINT32_MAX-249,v);h.observe(250,v);
    TEST_ASSERT_EQUAL(2,h.size());TEST_ASSERT_TRUE(h.at(1).valid&1);
    h.observe(70000,v);TEST_ASSERT_EQUAL(1,h.size());
}
void test_gap_between_samples(){
    DiagnosticHistory h;TelemetryValues v;v.set(Channel::BusL,30,true);v.set(Channel::BusR,40,true);
    h.observe(0,v);v.set(Channel::BusL,0,false);h.observe(100,v);
    v.set(Channel::BusL,35,true);h.observe(500,v);
    TEST_ASSERT_TRUE(h.at(1).broken&(1u<<unsigned(Channel::BusL)));
    TEST_ASSERT_FALSE(h.at(1).broken&(1u<<unsigned(Channel::BusR)));
    v.set(Channel::BusL,std::numeric_limits<float>::quiet_NaN(),true);
    TEST_ASSERT_FALSE(v.has(Channel::BusL));
}
void test_connection_edges(){
    DiagnosticHistory h;h.connection(LinkId::Vcu,false,0);
    TEST_ASSERT_EQUAL(0,h.event_count());
    h.connection(LinkId::Vcu,true,100);h.connection(LinkId::Vcu,true,500);
    TEST_ASSERT_EQUAL(100,h.connection(LinkId::Vcu).since);
    h.connection(LinkId::Vcu,false,900);h.connection(LinkId::Vcu,false,1000);
    TEST_ASSERT_EQUAL(1,h.connection(LinkId::Vcu).drops);
    h.connection(LinkId::Vcu,true,1200);
    TEST_ASSERT_EQUAL(1200,h.connection(LinkId::Vcu).since);
    TEST_ASSERT_TRUE(h.event(0).kind==EventKind::Restored);
}
void test_faults_not_cleared_on_disconnect(){
    DiagnosticHistory h;h.fault(LinkId::MotorL,3,true,100);
    TEST_ASSERT_EQUAL(2,h.connection(LinkId::MotorL).faults);
    h.fault(LinkId::MotorL,3,true,200);h.fault(LinkId::MotorL,0,false,300);
    TEST_ASSERT_EQUAL(2,h.event_count());
    h.fault(LinkId::MotorL,0,true,400);
    TEST_ASSERT_EQUAL(4,h.event_count());TEST_ASSERT_TRUE(h.event(0).kind==EventKind::FaultOff);
}
void test_events_bounded(){
    DiagnosticHistory h;
    for(unsigned i=0;i<80;++i) h.connection(LinkId::Vcu,i%2==0,i*100);
    TEST_ASSERT_EQUAL(32,h.event_count());TEST_ASSERT_EQUAL(48,h.event_overwrites());
    TEST_ASSERT_EQUAL(7900,h.event(0).ms);
}
void test_rtk_steps_and_baseline(){
    DiagnosticHistory h;TelemetryValues v;v.rtk=2;h.observe(100,v);
    v.rtk=4;h.observe(1000,v);v.rtk=5;h.observe(2000,v);
    h.observe(61500,v);
    TEST_ASSERT_EQUAL(2,h.rtk_count());TEST_ASSERT_EQUAL(4,h.rtk_at(0).state);
    TEST_ASSERT_EQUAL(5,h.rtk_at(1).state);
}
void test_rtk_bounded(){
    DiagnosticHistory h;TelemetryValues v;
    for(unsigned i=0;i<300;++i){v.rtk=i%2;h.observe(i*20,v);}
    TEST_ASSERT_EQUAL(256,h.rtk_count());TEST_ASSERT_EQUAL(44,h.rtk_overwrites());
}
void test_receiver_contract(){
    CarCheckReceiver r;
    uint8_t steering[]={0x0C,0xFE,0,0,0,0,0x81,42};
    TEST_ASSERT_TRUE(r.receive(0x1804C0D0,steering,8,true,false,10));
    TEST_ASSERT_FLOAT_WITHIN(.0001,-.5,r.steering.unit);TEST_ASSERT_TRUE(r.steering.valid);
    uint8_t imu[]={0x2E,0xFB,0x19,0,0xCE,0xFF,0x83,8};
    r.receive(0x1805C0D0,imu,8,true,false,10);
    TEST_ASSERT_FLOAT_WITHIN(.001,-12.34,r.imu.yaw_dps);
    TEST_ASSERT_FLOAT_WITHIN(.001,.25,r.imu.ax_g);TEST_ASSERT_FLOAT_WITHIN(.001,-.5,r.imu.ay_g);
    uint8_t wheels[]={0,0,0xFF,0xFF,0xD2,4,100,0};
    r.receive(0x1806C0D0,wheels,8,true,false,10);
    TEST_ASSERT_TRUE(r.wheels.valid[0]);TEST_ASSERT_FALSE(r.wheels.valid[1]);
    TEST_ASSERT_FLOAT_WITHIN(.001,123.4,r.wheels.kph[2]);
    uint8_t control[]={1,7,0x79,0x20,0x12,0,0,99};
    r.receive(0x1807C0D0,control,8,true,false,10);
    TEST_ASSERT_TRUE(r.control.tv_requested&&r.control.regen_requested&&r.control.paddock_requested);
    TEST_ASSERT_TRUE(r.control.tv_active&&r.control.paddock_active&&r.control.output_allowed&&r.control.cluster_fresh&&r.control.brake_installed);
    TEST_ASSERT_FALSE(r.control.regen_active);TEST_ASSERT_EQUAL(0x12,r.control.regen_block);
}
void test_reject_malformed_frames(){
    CarCheckReceiver r;uint8_t b[8]{};
    TEST_ASSERT_FALSE(r.receive(0x1804C0D0,b,7,true,false,1));
    TEST_ASSERT_FALSE(r.receive(0x1804C0D0,b,8,false,false,1));
    TEST_ASSERT_FALSE(r.receive(0x1804C0D0,b,8,true,true,1));
    TEST_ASSERT_FALSE(r.receive(0x1804C0D0,nullptr,8,true,false,1));
    TEST_ASSERT_FALSE(r.receive(0x1808C0D0,b,8,true,false,1));
    TEST_ASSERT_FALSE(r.steering_rx.seen);
}
void test_quality_legacy_invalid_stale(){
    using namespace car_check;
    Reception r;TEST_ASSERT_TRUE(r.quality(0,true)==Quality::Missing);
    r.received(UINT32_MAX-49);
    TEST_ASSERT_TRUE(r.quality(250,true)==Quality::Valid);
    TEST_ASSERT_TRUE(r.quality(251,true)==Quality::Stale);
    r.received(500);TEST_ASSERT_TRUE(r.quality(500,true,false)==Quality::Unsupported);
    TEST_ASSERT_TRUE(r.quality(500,false)==Quality::Invalid);
    uint8_t b[]={0xE9,3,0,0,0,0,0x81,0};
    TEST_ASSERT_FALSE(decode_steering(b).valid);
}
void test_navigation_and_marker_contract(){
    CheckUi u;u.tap(80,40,false);TEST_ASSERT_TRUE(u.page==CheckPage::Graph);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Home);
    u.tap(50,120,true);TEST_ASSERT_TRUE(u.page==CheckPage::Warning);
    u.tap(230,220,true);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(50,70,false);TEST_ASSERT_TRUE(u.page==CheckPage::Motor);
    const CheckPage pages[]={CheckPage::Motor,CheckPage::Power,CheckPage::Vcu,CheckPage::Gps};
    int count=0;
    for(auto p:pages) for(int row=0;row<(p==CheckPage::Motor?10:12);++row) {
        GraphKind kind;
        if(!check_graph_for_row(p,row,kind)) continue;
        ++count;u.page=p;u.tap(170,p==CheckPage::Motor?58+row*15:45+row*14,false);
        TEST_ASSERT_TRUE(u.page==CheckPage::Graph);TEST_ASSERT_TRUE(u.graph==kind);
        u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==p);
    }
    TEST_ASSERT_EQUAL(9,count);
    u.page=CheckPage::Vcu;u.tap(170,158,false);TEST_ASSERT_TRUE(u.page==CheckPage::Sensors);
    u.tap(270,16,false);u.tap(170,119,false);TEST_ASSERT_TRUE(u.page==CheckPage::Control);
    u.tap(170,201,false);TEST_ASSERT_TRUE(u.page==CheckPage::Reasons);
    u.home();TEST_ASSERT_TRUE(u.page==CheckPage::Home);
}
void test_marker_pixels_match_touch(){
    CheckSnapshot d;DiagnosticHistory h;TelemetryValues v;FrameBuffer f;CheckUi u;
    for(auto p:{CheckPage::Motor,CheckPage::Power,CheckPage::Vcu,CheckPage::Gps,CheckPage::Sensors}) {
        u.page=p;f.clear();check_draw(f,u,d,h,v,600000);
        for(int r=0;r<(p==CheckPage::Motor?10:12);++r) {
            GraphKind k;bool clickable=check_graph_for_row(p,r,k),ink=false;
            int y=p==CheckPage::Motor?58+r*15:45+r*14;
            for(int a=1;a<=5;++a)for(int b=y;b<y+7;++b)ink|=f.get(a,b);
            TEST_ASSERT_EQUAL(clickable,ink);
        }
    }
}
void test_required_glyphs_exist(){for(char c:std::string("*>?|_"))TEST_ASSERT_NOT_NULL(font_glyph(c));}
void test_disconnected_not_zero_ok(){
    CheckSnapshot d;check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("-- WAIT",d.motor[1][1]);TEST_ASSERT_EQUAL_STRING("-- WAIT",d.motor[8][2]);
    TEST_ASSERT_EQUAL_STRING("-- WAIT",d.power[2][1]);TEST_ASSERT_EQUAL_STRING("-- WAIT",d.vcu[4][1]);
    TEST_ASSERT_EQUAL_STRING("-- -",d.sensors[0][1]);TEST_ASSERT_FALSE(d.control_valid);
    HomeData home=check_home_snapshot(600000);TEST_ASSERT_EQUAL(-1,home.soc);TEST_ASSERT_FALSE(home.bms_ok);
}
void test_independent_motor_frames(){
    state.controller_l_fb1_last_ms=599900;
    CheckSnapshot d;check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("0.0 V",d.motor[1][1]);TEST_ASSERT_EQUAL_STRING("OK / D0",d.motor[5][1]);
    TEST_ASSERT_EQUAL_STRING("-- WAIT",d.motor[6][1]);
    TEST_ASSERT_EQUAL_STRING("-- WAIT",d.motor[1][2]);
}
void test_drivetrain_latch_snapshot_and_history(){
    state.controller_r_fb1_last_ms=599900;
    state.drivetrain_fault_r=1;state.drivetrain_dropouts_r=3;
    CheckSnapshot d;check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("DROPOUT 3",d.motor[5][2]);
    DiagnosticHistory h;
    h.drivetrain(1,LinkId::MotorR,EventKind::DriveDropout,599000);
    h.drivetrain(1,LinkId::MotorR,EventKind::DriveDropout,599000);
    TEST_ASSERT_EQUAL(1,h.event_count());
    TEST_ASSERT_TRUE(h.event(0).kind==EventKind::DriveDropout);
    TEST_ASSERT_TRUE(h.event(0).node==LinkId::MotorR);
}
void test_wss_never_rpm_fallback(){
    state.vehicle_speed_kph=40;state.vehicle_speed_valid=true;
    CheckSnapshot d;check_snapshot(d,600000);TEST_ASSERT_EQUAL_STRING("-- WAIT",d.vcu[4][1]);
    state.wss_kph=12;state.wss_valid=true;state.vehicle_speed_last_rx_ms=600000;
    check_snapshot(d,600000);TEST_ASSERT_EQUAL_STRING("12.0 km/h",d.vcu[4][1]);
}
void test_home_speed_holds_through_short_invalid(){
    // 차속 프레임은 계속 오는데 valid만 잠깐 0: 직전 값 유지, 1초 넘으면 '--'.
    state.wss_kph=42;state.wss_valid=false;state.vehicle_speed_last_rx_ms=600000;
    state.wss_last_valid_ms=599500;
    HomeData d=check_home_snapshot(600000);TEST_ASSERT_TRUE(d.speed_ok);TEST_ASSERT_FLOAT_WITHIN(.01,42,d.speed);
    state.wss_last_valid_ms=598900;
    d=check_home_snapshot(600000);TEST_ASSERT_FALSE(d.speed_ok);
    // valid였어도 프레임 자체가 끊기면 '--'.
    state.wss_valid=true;state.wss_last_valid_ms=599400;state.vehicle_speed_last_rx_ms=599400;
    d=check_home_snapshot(600000);TEST_ASSERT_FALSE(d.speed_ok);
}
void test_requests_distinct_and_no_pressure(){
    state.tc_enabled=true;state.regen_level=1;
    uint8_t b[]={1,0,0,1,1,0,0,1};state.car_check.receive(car_check::CONTROL_ID,b,8,true,false,600000);
    CheckSnapshot d;check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("ON / OFF",d.vcu[5][1]);TEST_ASSERT_EQUAL_STRING("ON / OFF",d.control[0][1]);
    TEST_ASSERT_EQUAL_STRING("-- WAIT",d.vcu[11][1]);
    TEST_ASSERT_EQUAL_STRING("NOT PROVIDED",d.sensors[10][1]);
    state.start_input_valid=true;state.start_input_present=true;state.start_input_mv=2250;
    check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("ON / ADC 2.25V",d.vcu[11][1]);
    b[0]=2;state.car_check.receive(car_check::CONTROL_ID,b,8,true,false,600000);check_snapshot(d,600000);
    TEST_ASSERT_FALSE(d.control_valid);TEST_ASSERT_EQUAL_STRING("-- UNKNOWN",d.control[1][1]);
}
void test_sensor_quality_not_numeric_legacy(){
    uint8_t b[]={0xF4,1,0,0,0,0,1,3};state.car_check.receive(car_check::STEERING_ID,b,8,true,false,600000);
    CheckSnapshot d;check_snapshot(d,600000);TEST_ASSERT_EQUAL_STRING("-- UNKNOWN",d.sensors[4][1]);
    b[6]=0x81;state.car_check.receive(car_check::STEERING_ID,b,8,true,false,600000);check_snapshot(d,600000);
    TEST_ASSERT_EQUAL_STRING("0.500 OK",d.sensors[4][1]);check_snapshot(d,600301);
    TEST_ASSERT_EQUAL_STRING("-- STALE",d.sensors[4][1]);
}
void test_observer_fix_and_rtcm_independent(){
    DiagnosticHistory h;TelemetryValues v;mock_gga=599000;mock_rtcm=590000;
    check_observe(h,v,600000);TEST_ASSERT_EQUAL(5,v.rtk);
    TEST_ASSERT_TRUE(h.connection(LinkId::Ntrip).live);TEST_ASSERT_FALSE(h.connection(LinkId::Rtcm).live);
    mock_quality=5;check_observe(h,v,600020);TEST_ASSERT_EQUAL(4,v.rtk);
    check_observe(h,v,603000);TEST_ASSERT_EQUAL(0,v.rtk);
    state.gps_last_rx_ms=603000;
    CheckSnapshot d;check_snapshot(d,603000);TEST_ASSERT_EQUAL_STRING("-- WAIT",d.gps[0][1]);
}
void test_graph_voltage_scaling_and_back_button(){
    DiagnosticHistory h;TelemetryValues v;CheckSnapshot d;FrameBuffer f;CheckUi u;
    v.set(Channel::BmsV,53.4,true);h.observe(1000,v);v.set(Channel::BmsV,53.6,true);h.observe(1500,v);
    u.page=CheckPage::Power;u.open_graph(GraphKind::BmsV);f.clear();check_draw(f,u,d,h,v,1500);
    TEST_ASSERT_GREATER_THAN(51,u.graph_min);TEST_ASSERT_LESS_THAN(56,u.graph_max);
    TEST_ASSERT_TRUE(f.get(248,2));TEST_ASSERT_TRUE(f.get(317,31));
    u.tap(270,15,false);TEST_ASSERT_TRUE(u.page==CheckPage::Power);
    u.page=CheckPage::Links;f.clear();check_draw(f,u,d,h,v,1500);
    TEST_ASSERT_TRUE(f.get(248,2));TEST_ASSERT_TRUE(f.get(0,37));TEST_ASSERT_TRUE(f.get(0,218));
}
void test_bms_home_and_graph_sources(){
    state.soc=.7;state.soc_valid=true;state.bms_ble_connected=true;state.bms_last_rx_ms=600000;
    state.bms_pack_voltage=53.7f;state.bms_current=-10.0f;
    state.current_lap_battery_used_x10=20;state.current_lap_battery_valid=true;
    state.last_lap_battery_used_x10=30;state.last_lap_battery_valid=true;
    HomeData d=check_home_snapshot(600000);TEST_ASSERT_EQUAL(70,d.soc);
    TEST_ASSERT_TRUE(d.bms_ok);TEST_ASSERT_FLOAT_WITHIN(.001,53.7,d.hv);
    TEST_ASSERT_TRUE(d.lap_battery_valid);TEST_ASSERT_EQUAL_UINT16(20,d.lap_battery_x10);
    TEST_ASSERT_TRUE(d.last_lap_battery_valid);TEST_ASSERT_EQUAL_UINT16(30,d.last_lap_battery_x10);
    DiagnosticHistory h;TelemetryValues v;check_observe(h,v,600000);
    TEST_ASSERT_FLOAT_WITHIN(.001,53.7,v.get(Channel::BmsV));
    TEST_ASSERT_FLOAT_WITHIN(.001,-10.0,v.get(Channel::BmsA));
    TEST_ASSERT_FLOAT_WITHIN(.001,-.537,v.get(Channel::BmsPower));
    d=check_home_snapshot(605001);TEST_ASSERT_FALSE(d.bms_ok);
}
void test_home_lv_voltage_validity_and_alignment(){
    HomeData d=check_home_snapshot(600000);
    TEST_ASSERT_FALSE(d.lv_ok);
    state.vcu_lv_seen=true;state.vcu_lv_last_ms=600000;
    state.vcu_lv_supply=lv_monitor::Sample(1200,2791,true,42);
    d=check_home_snapshot(600500);
    TEST_ASSERT_TRUE(d.lv_ok);TEST_ASSERT_FLOAT_WITHIN(.001,12.0,d.lv);
    FrameBuffer actual,expected;actual.clear();expected.clear();
    check_home_draw(actual,d,false);fb_text(expected,232,214,"12.00 V",2);
    for(int y=214;y<228;++y)for(int x=220;x<320;++x)
        TEST_ASSERT_EQUAL(expected.get(x,y),actual.get(x,y));
    d=check_home_snapshot(600501);TEST_ASSERT_FALSE(d.lv_ok);
    actual.clear();expected.clear();check_home_draw(actual,d,false);
    fb_text(expected,268,214,"-- V",2);
    for(int y=214;y<228;++y)for(int x=220;x<320;++x)
        TEST_ASSERT_EQUAL(expected.get(x,y),actual.get(x,y));
    state.vcu_lv_supply.valid=false;
    TEST_ASSERT_FALSE(check_home_snapshot(600000).lv_ok);
    state.vcu_lv_supply=lv_monitor::Sample(0,0,true,43);
    d=check_home_snapshot(600000);
    TEST_ASSERT_TRUE(d.lv_ok);TEST_ASSERT_FLOAT_WITHIN(.001,0,d.lv);
}
void test_home_brake_status(){
    state.vcu_cluster_status_last_ms=600000;state.brake=true;
    HomeData d=check_home_snapshot(600000);
    TEST_ASSERT_TRUE(d.brake_valid);TEST_ASSERT_TRUE(d.brake_active);
    FrameBuffer f;f.clear();check_home_draw(f,d,false);
    bool active_ink=false;for(int x=262;x<280;++x)for(int y=44;y<51;++y)active_ink|=f.get(x,y);
    TEST_ASSERT_TRUE(active_ink);
    d=check_home_snapshot(600301);
    TEST_ASSERT_FALSE(d.brake_valid);TEST_ASSERT_FALSE(d.brake_active);
    f.clear();check_home_draw(f,d,false);
    bool stale_ink=false;for(int x=262;x<280;++x)for(int y=44;y<51;++y)stale_ink|=f.get(x,y);
    TEST_ASSERT_FALSE(stale_ink);
}
void test_home_hit_areas(){
    CheckUi u;
    u.tap(260,111,false);TEST_ASSERT_TRUE(u.page==CheckPage::Graph);
    TEST_ASSERT_TRUE(u.graph==GraphKind::Wss);TEST_ASSERT_TRUE(u.graph_origin==CheckPage::Home);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Home);
    u.tap(261,107,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.home();u.tap(0,138,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.home();u.tap(260,183,false);TEST_ASSERT_TRUE(u.page==CheckPage::Graph);
    TEST_ASSERT_TRUE(u.graph==GraphKind::LapBattery);
    u.tap(80,120,false);TEST_ASSERT_EQUAL_UINT(1,u.list_page);
    u.tap(220,120,false);TEST_ASSERT_EQUAL_UINT(0,u.list_page);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Home);
    u.home();u.tap(20,120,true);TEST_ASSERT_TRUE(u.page==CheckPage::Warning);
}
void test_back_and_car_check_navigation(){
    CheckUi u;
    u.tap(20,120,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(80,80,false);TEST_ASSERT_TRUE(u.page==CheckPage::Motor);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(240,80,false);TEST_ASSERT_TRUE(u.page==CheckPage::Power);
    u.tap(170,45+2*14,false);TEST_ASSERT_TRUE(u.page==CheckPage::Graph);
    TEST_ASSERT_TRUE(u.graph==GraphKind::BmsV);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Power);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(80,170,false);TEST_ASSERT_TRUE(u.page==CheckPage::Vcu);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(240,170,false);TEST_ASSERT_TRUE(u.page==CheckPage::Gps);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Menu);
    u.tap(270,16,false);TEST_ASSERT_TRUE(u.page==CheckPage::Home);
}
void test_detail_rows_have_no_separators(){
    CheckSnapshot d;DiagnosticHistory h;TelemetryValues v;FrameBuffer f;CheckUi u;
    u.page=CheckPage::Power;f.clear();check_draw(f,u,d,h,v,600000);
    for(int y=56;y<=210;y+=14) {
        int lit=0;for(int x=8;x<312;++x) lit+=f.get(x,y)?1:0;
        TEST_ASSERT_LESS_THAN(250,lit);
    }
}
void write_image(const char *name,const FrameBuffer &f,bool red=false){
    std::filesystem::create_directories(".tmp");char path[128];std::snprintf(path,sizeof(path),".tmp/ui_%s.ppm",name);
    FILE *out=std::fopen(path,"wb");TEST_ASSERT_NOT_NULL(out);std::fprintf(out,"P6\n320 240\n255\n");
    for(int y=0;y<240;++y)for(int x=0;x<320;++x){unsigned char p=f.get(x,y)?255:0;unsigned char rgb[]={static_cast<unsigned char>(red?255:p),p,p};std::fwrite(rgb,1,3,out);}
    std::fclose(out);
}
void fixture(){
    state.controller_l_fb1_last_ms=state.controller_l_fb2_last_ms=599950;
    state.controller_r_fb1_last_ms=state.controller_r_fb2_last_ms=599950;
    state.bus_voltage=53.7f;state.bus_voltage_r=53.6f;state.bus_current=24.1f;state.bus_current_r=23.9f;
    state.phase_current=62;state.phase_current_r=61;state.speed_rpm_l=2400;state.speed_rpm_r=2390;
    state.motor_temp=54;state.motor_temp_r=53;state.controller_temp=42;state.controller_temp_r=43;
    state.bms_ble_connected=true;state.bms_last_rx_ms=599900;state.soc_valid=true;state.soc=.78f;
    state.bms_pack_voltage=53.7f;state.bms_current=48.2f;state.bms_temp_c=28;
    state.bms_remaining_mah=23000;state.bms_soh=98;state.bms_cycles=42;
    state.vcu_lv_seen=true;state.vcu_lv_last_ms=599950;
    state.vcu_lv_supply=lv_monitor::Sample(1200,2791,true,42);
    state.em_record_seen=true;state.em_record_last_ms=599990;state.em_hv_decivolts=537;
    state.em_lv_centivolts=1342;state.em_current_deciamps=482;state.em_cpu_centidegrees=3150;
    state.gear=2;state.gear_from_can=true;state.brake=true;state.hv_active=true;state.vcu_cluster_status_last_ms=599950;
    state.throttle_pct=32;state.throttle_valid=true;state.throttle_last_rx_ms=599950;
    state.tc_enabled=true;state.regen_level=1;state.paddock=true;state.paddock_active=true;
    state.wss_kph=32;state.wss_valid=true;state.vehicle_speed_last_rx_ms=599950;
    state.gps_fix_ok=true;state.gps_last_rx_ms=599950;state.gps_latitude=37.2951234;state.gps_longitude=126.9756789;
    state.gps_pps_last_ms=599900;state.lap_count=3;state.best_lap_count=1;state.current_lap_ms=85670;state.last_lap_ms=82770;state.best_lap_ms=80770;
    state.current_lap_battery_used_x10=10;state.current_lap_battery_valid=true;
    state.last_lap_battery_used_x10=20;state.last_lap_battery_valid=true;
    state.lap_battery_used_x10[0]=20;state.lap_battery_valid[0]=true;
    state.lap_battery_used_x10[1]=30;state.lap_battery_valid[1]=true;
    state.lap_battery_used_x10[2]=20;state.lap_battery_valid[2]=true;
    uint8_t steer[]={0xFA,0,0,0,0,0,0x81,45},imu[]={0xE2,4,25,0,0xCE,0xFF,0x83,45};
    uint8_t wheels[]={0x3C,1,0x40,1,0x39,1,0x42,1},control[]={1,7,0x39,0,4,0,0,45};
    state.car_check.receive(car_check::STEERING_ID,steer,8,true,false,599950);
    state.car_check.receive(car_check::IMU_ID,imu,8,true,false,599950);
    state.car_check.receive(car_check::WHEELS_ID,wheels,8,true,false,599950);
    state.car_check.receive(car_check::CONTROL_ID,control,8,true,false,599950);
}
void test_render_production_screens(){
    fixture();CheckSnapshot d;check_snapshot(d,600000);DiagnosticHistory h;TelemetryValues v;
    for(uint32_t t=540000;t<=600000;t+=20){
        float wave=std::sin((t-540000)/4000.0f);
        v.set(Channel::Wss,35+20*wave,true);
        v.set(Channel::BusL,35+28*wave,true);v.set(Channel::BusR,31+25*wave,!(t>575000&&t<579000));
        v.set(Channel::PhaseL,75+60*wave,true);v.set(Channel::PhaseR,65+50*wave,true);
        v.set(Channel::VoltL,54-1.5f*wave,true);v.set(Channel::VoltR,53.5f-wave,true);
        v.set(Channel::BmsV,53.7f-wave,true);v.set(Channel::BmsA,40+60*wave,true);
        v.set(Channel::BmsPower,(53.7f-wave)*(40+60*wave)/1000.0f,true);
        v.set(Channel::Throttle,45+35*wave,true);
        v.rtk=t<550000?2:t<560000?4:t<575000?5:t<582000?0:5;h.observe(t,v);
    }
    for(size_t i=0;i<LINKS;++i)h.connection(static_cast<LinkId>(i),true,540000);
    h.connection(LinkId::MotorR,false,575000);h.connection(LinkId::MotorR,true,579000);
    h.fault(LinkId::MotorL,1u<<5,true,580000);h.fault(LinkId::MotorL,0,true,584000);
    FrameBuffer f;f.clear();check_home_draw(f,check_home_snapshot(600000),false);write_image("home",f);
    CheckUi u;
    const CheckPage pages[]={CheckPage::Menu,CheckPage::Motor,CheckPage::Power,CheckPage::Vcu,CheckPage::Gps,CheckPage::Sensors,CheckPage::Control,CheckPage::Reasons,CheckPage::Links,CheckPage::Events};
    const char *names[]={"menu","motor","power","vcu","gps","sensors","control","reasons","links","events"};
    for(int i=0;i<10;++i){u.page=pages[i];u.list_page=0;f.clear();check_draw(f,u,d,h,v,600000);write_image(names[i],f);}
    u.page=CheckPage::Links;u.list_page=1;f.clear();check_draw(f,u,d,h,v,600000);write_image("links_2",f);
    u.page=CheckPage::Reasons;u.list_page=1;f.clear();check_draw(f,u,d,h,v,600000);write_image("reasons_2",f);
    const GraphKind kinds[]={GraphKind::Wss,GraphKind::Bus,GraphKind::Phase,GraphKind::MotorV,GraphKind::BmsV,GraphKind::BmsA,GraphKind::BmsPower,GraphKind::Throttle,GraphKind::Rtk,GraphKind::LapBattery};
    const char *gnames[]={"graph_wss","graph_bus","graph_phase","graph_motor_v","graph_bms_v","graph_bms_a","graph_bms_power","graph_throttle","graph_rtk","graph_lap_battery"};
    for(int i=0;i<10;++i){u.page=CheckPage::Power;u.open_graph(kinds[i]);f.clear();check_draw(f,u,d,h,v,600000);write_image(gnames[i],f);}
    d.warnings[0]="L MOTOR HOT";d.warnings[1]="R CAN TIMEOUT";d.warning_count=2;
    u.page=CheckPage::Warning;f.clear();check_draw(f,u,d,h,v,600000);write_image("warning",f,true);
    state=ClusterState{};check_snapshot(d,600000);u.page=CheckPage::Motor;f.clear();check_draw(f,u,d,h,v,600000);write_image("motor_wait",f,true);
    f.clear();check_home_draw(f,check_home_snapshot(600000),true);write_image("home_wait",f,true);
    HomeData max;max.speed_ok=max.bms_ok=max.throttle_ok=true;max.speed=6553.4f;max.hv=999.9f;max.soc=100;max.throttle=100;
    max.lv_ok=true;max.lv=655.35f;
    max.lap=255;max.lap_ms=UINT32_MAX;max.best_ms=UINT32_MAX;
    f.clear();check_home_draw(f,max,false);write_image("home_limits",f);
    std::printf("DiagnosticHistory host bytes: %zu\n",sizeof(h));
}
int main(int,char**){
    UNITY_BEGIN();
    RUN_TEST(test_history_rate_and_retention);RUN_TEST(test_history_wrap_and_no_backfill);RUN_TEST(test_gap_between_samples);
    RUN_TEST(test_connection_edges);RUN_TEST(test_faults_not_cleared_on_disconnect);RUN_TEST(test_events_bounded);
    RUN_TEST(test_rtk_steps_and_baseline);RUN_TEST(test_rtk_bounded);
    RUN_TEST(test_receiver_contract);RUN_TEST(test_reject_malformed_frames);RUN_TEST(test_quality_legacy_invalid_stale);
    RUN_TEST(test_navigation_and_marker_contract);RUN_TEST(test_marker_pixels_match_touch);RUN_TEST(test_required_glyphs_exist);
    RUN_TEST(test_disconnected_not_zero_ok);RUN_TEST(test_independent_motor_frames);RUN_TEST(test_drivetrain_latch_snapshot_and_history);RUN_TEST(test_wss_never_rpm_fallback);RUN_TEST(test_home_speed_holds_through_short_invalid);
    RUN_TEST(test_requests_distinct_and_no_pressure);RUN_TEST(test_sensor_quality_not_numeric_legacy);
    RUN_TEST(test_observer_fix_and_rtcm_independent);RUN_TEST(test_graph_voltage_scaling_and_back_button);
    RUN_TEST(test_bms_home_and_graph_sources);RUN_TEST(test_home_lv_voltage_validity_and_alignment);RUN_TEST(test_home_brake_status);RUN_TEST(test_home_hit_areas);RUN_TEST(test_back_and_car_check_navigation);RUN_TEST(test_detail_rows_have_no_separators);RUN_TEST(test_render_production_screens);
    return UNITY_END();
}
