#include "diagnostics.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
namespace {
void bar(FrameBuffer &f,int x,int y,int w,int height,float ratio,bool valid) {
    fb_rect(f,x,y,w,height,false,true);
    if(valid) fb_rect(f,x+2,y+2,static_cast<int>((w-4)*std::max(0.0f,std::min(1.0f,ratio))),height-4,true,true);
}
void power_bar(FrameBuffer &f,int x,int y,int w,int height,float ratio,bool charging,bool valid) {
    fb_rect(f,x,y,w,height,false,true);
    const int center=x+w/2;
    fb_vline(f,center,y,height,true);
    if(!valid) return;
    const float bounded=std::max(0.0f,std::min(1.0f,ratio));
    const int half=center-(x+2);
    const int fill=static_cast<int>(half*bounded);
    if(fill<=0) return;
    if(charging) fb_rect(f,center-fill,y+2,fill,height-4,true,true);
    else fb_rect(f,center+1,y+2,fill,height-4,true,true);
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
    const char *gear=d.gear_ok?(d.gear==1?"R":d.gear==2?"D":d.gear==3?"P":"N"):"-";
    right_text(f,316,2,gear,5);
    right_text(f,316,58,"HV SOC");fb_rect(f,293,69,23,62,false,true);
    if(d.soc>=0) {int fill=std::max(0,std::min(100,d.soc))*58/100;fb_rect(f,295,129-fill,19,fill,true,true);}
    if(d.soc>=0) std::snprintf(b,sizeof(b),"%d%%",d.soc);else std::snprintf(b,sizeof(b),"--%%");
    int pct_x=304-static_cast<int>(std::strlen(b))*6;
    fb_text(f,pct_x,134,b,2);
    right_text(f,316,162,"HV");right_text(f,316,190,"LV");
    if(d.em_ok&&d.hv>=0.0f&&d.hv<1000.0f) std::snprintf(b,sizeof(b),"%.1f V",d.hv);else if(d.em_ok) std::snprintf(b,sizeof(b),"ERR V");else std::snprintf(b,sizeof(b),"-- V");
    right_text(f,316,173,b);
    if(d.em_ok&&d.lv>=0.0f&&d.lv<100.0f) std::snprintf(b,sizeof(b),"%.2f V",d.lv);else if(d.em_ok) std::snprintf(b,sizeof(b),"ERR V");else std::snprintf(b,sizeof(b),"-- V");
    right_text(f,316,201,b);
    if(d.throttle_ok) std::snprintf(b,sizeof(b),"THR %.0f%%",d.throttle);else std::snprintf(b,sizeof(b),"THR --");
    fb_text(f,8,116,b,1);bar(f,86,113,172,14,d.throttle/100,d.throttle_ok);
    fb_text(f,8,142,"BRK --",1);bar(f,86,139,172,14,0,false);
    if(d.em_ok) std::snprintf(b,sizeof(b),"%.2f kW",d.power_kw);else std::snprintf(b,sizeof(b),"-- kW");
    fb_text(f,8,168,b,1);power_bar(f,86,165,172,14,d.power_kw/10,d.charging,d.em_ok);
    fb_text(f,86,181,"CHR",1);fb_text(f,169,181,"0",1);right_text(f,258,181,"PWR");
    std::snprintf(b,sizeof(b),"L%u %lu:%02lu.%02lu",d.lap,static_cast<unsigned long>(d.lap_ms/60000),
        static_cast<unsigned long>(d.lap_ms/1000%60),static_cast<unsigned long>(d.lap_ms/10%100));
    fb_text(f,8,199,b,2);
    std::snprintf(b,sizeof(b),"BEST %u %lu:%02lu.%02lu",d.best_lap,static_cast<unsigned long>(d.best_ms/60000),
        static_cast<unsigned long>(d.best_ms/1000%60),static_cast<unsigned long>(d.best_ms/10%100));
    fb_text(f,8,222,b,1);
}
