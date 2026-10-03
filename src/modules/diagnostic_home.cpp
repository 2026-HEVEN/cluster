#include "diagnostics.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
namespace {
void bar(FrameBuffer &f,int x,int y,int w,int height,float ratio,bool valid) {
    fb_rect(f,x,y,w,height,false,true);
    if(valid) fb_rect(f,x+2,y+2,static_cast<int>((w-4)*std::max(0.0f,std::min(1.0f,ratio))),height-4,true,true);
}
void right_text(FrameBuffer &f,int right,int y,const char *text,int scale=1) {
    fb_text(f,right-static_cast<int>(std::strlen(text))*6*scale,y,text,scale);
}
}
void check_home_draw(FrameBuffer &f,const HomeData &d,bool warning) {
    char b[40];
    if(d.speed_ok) std::snprintf(b,sizeof(b),"%.0f",d.speed);else std::snprintf(b,sizeof(b),"--");
    const int scale=std::strlen(b)>3?8:14;
    fb_text(f,4,0,b,scale);right_text(f,260,100,"km/h");
    const char *gear="-";
    if(d.gear_ok) gear=d.gear==0?"N":d.gear==1?"R":d.gear==2?"D":"-";
    right_text(f,316,2,gear,5);
    if(d.brake_valid&&d.brake_active) fb_text(f,262,44,"BRK",1);
    right_text(f,316,58,"HV SOC");fb_rect(f,293,69,23,62,false,true);
    if(d.soc>=0) {int fill=std::max(0,std::min(100,d.soc))*58/100;fb_rect(f,295,129-fill,19,fill,true,true);}
    if(d.soc>=0) std::snprintf(b,sizeof(b),"%d%%",d.soc);else std::snprintf(b,sizeof(b),"--%%");
    int pct_x=304-static_cast<int>(std::strlen(b))*6;
    fb_text(f,pct_x,134,b,2);
    right_text(f,316,162,"HV PACK");
    if(d.bms_ok&&d.hv>=0.0f&&d.hv<1000.0f) std::snprintf(b,sizeof(b),"%.1f V",d.hv);else if(d.bms_ok) std::snprintf(b,sizeof(b),"ERR V");else std::snprintf(b,sizeof(b),"-- V");
    right_text(f,316,177,b,2);
    if(d.throttle_ok) std::snprintf(b,sizeof(b),"THR %.0f%%",d.throttle);else std::snprintf(b,sizeof(b),"THR --");
    fb_text(f,8,128,b,1);bar(f,72,123,196,20,d.throttle/100,d.throttle_ok);
    std::snprintf(b,sizeof(b),"L%u %lu:%02lu.%02lu",d.lap,static_cast<unsigned long>(d.lap_ms/60000),
        static_cast<unsigned long>(d.lap_ms/1000%60),static_cast<unsigned long>(d.lap_ms/10%100));
    fb_text(f,8,166,b,2);
    if(d.last_lap_battery_valid) std::snprintf(b,sizeof(b),"LAST %.1f%%",d.last_lap_battery_x10/10.0f);
    else std::snprintf(b,sizeof(b),"LAST --%%");
    right_text(f,220,151,b,1);
    if(d.lap_battery_valid) std::snprintf(b,sizeof(b),"%.1f%%",d.lap_battery_x10/10.0f);
    else std::snprintf(b,sizeof(b),"--%%");
    fb_text(f,145,172,"NOW",1);
    right_text(f,220,168,b,2);
    std::snprintf(b,sizeof(b),"BEST %u %lu:%02lu.%02lu",d.best_lap,static_cast<unsigned long>(d.best_ms/60000),
        static_cast<unsigned long>(d.best_ms/1000%60),static_cast<unsigned long>(d.best_ms/10%100));
    fb_text(f,8,207,b,1);
}
