#include "diagnostics.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
namespace {
void line(FrameBuffer &f,int x0,int y0,int x1,int y1,bool dotted=false) {
    int dx=std::abs(x1-x0),sx=x0<x1?1:-1,dy=-std::abs(y1-y0),sy=y0<y1?1:-1,e=dx+dy;
    for(;;) {
        if(!dotted || (dx>=-dy?x0:y0)%6<3) f.pixel(x0,y0,true);
        if(x0==x1 && y0==y1) break;
        int e2=2*e;
        if(e2>=dy){e+=dy;x0+=sx;} if(e2<=dx){e+=dx;y0+=sy;}
    }
}
struct Spec { const char *name; Channel a,b; bool paired; };
Spec spec(GraphKind g) {
    switch(g) {
    case GraphKind::Bus: return {"BUS CURRENT A",Channel::BusL,Channel::BusR,true};
    case GraphKind::Phase: return {"PHASE CURRENT A",Channel::PhaseL,Channel::PhaseR,true};
    case GraphKind::MotorV: return {"MOTOR BUS V",Channel::VoltL,Channel::VoltR,true};
    case GraphKind::EmHv: return {"EM HV V",Channel::EmHv,Channel::EmHv,false};
    case GraphKind::EmLv: return {"EM LV V",Channel::EmLv,Channel::EmLv,false};
    case GraphKind::BmsV: return {"BMS PACK V",Channel::BmsV,Channel::BmsV,false};
    case GraphKind::EmA: return {"EM HV CURRENT A",Channel::EmA,Channel::EmA,false};
    case GraphKind::Throttle: return {"THROTTLE %",Channel::Throttle,Channel::Throttle,false};
    default: return {"WSS km/h",Channel::Wss,Channel::Wss,false};
    }
}
void header(FrameBuffer &f,const char *s) {
    fb_text(f,8,10,s,2);fb_rect(f,248,2,70,30,false,true);fb_text(f,255,10,"BACK",2);fb_hline(f,0,37,320,true);
    fb_vline(f,46,66,127,true);fb_hline(f,46,192,264,true);
    const char *labels[]={"-60s","-45s","-30s","-15s","0s"};
    for(int i=0;i<5;++i) fb_text(f,39+i*65,199,labels[i],1);
    fb_hline(f,0,218,320,true);
}
}
void diagnostic_graph(FrameBuffer &f,CheckUi &ui,const DiagnosticHistory &h,const TelemetryValues &v,uint32_t now) {
    if(ui.graph==GraphKind::Rtk) {
        header(f,"RTK HISTORY");fb_text(f,8,46,diagnostic_rtk_name(v.rtk),2);fb_text(f,266,48,"LIVE",1);
        const char *names[]={"NO DATA","NO FIX","GNSS","DGPS","FLOAT","FIXED","OTHER"};
        auto py=[](uint8_t s){return 188-std::min<int>(6,s)*19;};
        for(int i=0;i<7;++i) fb_text(f,0,py(i)-3,names[i],1);
        for(size_t i=0;i<h.rtk_count();++i) {
            const auto &a=h.rtk_at(i);
            int64_t t=-static_cast<int64_t>(now-a.ms);
            int64_t end=i+1<h.rtk_count()?-static_cast<int64_t>(now-h.rtk_at(i+1).ms):0;
            if(end < -60000) continue;
            int x0=46+static_cast<int>((std::max<int64_t>(-60000,t)+60000)*263/60000);
            int x1=46+static_cast<int>((std::max<int64_t>(-60000,end)+60000)*263/60000);
            line(f,x0,py(a.state),x1,py(a.state));
            if(i+1<h.rtk_count()) line(f,x1,py(a.state),x1,py(h.rtk_at(i+1).state));
        }
        char b[52]; std::snprintf(b,sizeof(b),"LIVE 60s / RTK overwritten %lu",static_cast<unsigned long>(h.rtk_overwrites()));
        fb_text(f,8,227,b,1);return;
    }
    const auto s=spec(ui.graph);header(f,s.name);
    const bool voltage=ui.graph==GraphKind::MotorV || ui.graph==GraphKind::EmHv ||
        ui.graph==GraphKind::EmLv || ui.graph==GraphKind::BmsV;
    float lo=0,hi=ui.graph==GraphKind::Throttle?100:1;
    bool found=false;
    for(size_t i=0;i<h.size();++i) {
        const auto &p=h.at(i);if(now-p.ms>60000) continue;
        for(Channel c:{s.a,s.b}) if(p.valid&(1u<<static_cast<unsigned>(c))) {
            if(voltage&&!found) lo=hi=p.value[static_cast<unsigned>(c)];
            lo=std::min(lo,p.value[static_cast<unsigned>(c)]);hi=std::max(hi,p.value[static_cast<unsigned>(c)]);
            found=true;
        }
    }
    if(voltage&&found) {
        const float margin=std::max(ui.graph==GraphKind::EmLv?.25f:1.0f,(hi-lo)*.1f);
        lo-=margin;hi+=margin;
    }
    const float step=std::pow(10.0f,std::floor(std::log10(std::max(1.0f,hi-lo))))/2;
    const float lower=std::floor(lo/step)*step,upper=std::ceil(hi/step)*step;
    ui.graph_min=ui.graph_range_set?std::min(ui.graph_min,lower):lower;
    ui.graph_max=ui.graph_range_set?std::max(ui.graph_max,upper):upper;
    if(found) ui.graph_range_set=true;
    auto py=[&](float n){return 188-static_cast<int>((n-ui.graph_min)*120/(ui.graph_max-ui.graph_min));};
    for(int i=0;i<3;++i) {
        char b[20];std::snprintf(b,sizeof(b),"%.1f",ui.graph_min+(ui.graph_max-ui.graph_min)*i/2);
        fb_text(f,0,184-i*60,b,1);
    }
    if(ui.graph_min<0) for(int x=47;x<310;x+=6) f.pixel(x,py(0),true);
    bool any=false;
    for(int side=s.paired?1:0;side>=0;--side) {
        Channel c=side?s.b:s.a;uint16_t mask=1u<<static_cast<unsigned>(c);
        int px=0,prev_y=0;uint32_t stamp=0;bool prev=false;
        for(size_t i=0;i<h.size();++i) {
            const auto &p=h.at(i);if(now-p.ms>60000) continue;
            if(!(p.valid&mask)){prev=false;continue;}
            int x=46+static_cast<int>((60000-(now-p.ms))*263ULL/60000),y=py(p.value[static_cast<unsigned>(c)]);
            bool dotted=s.paired&&side==0;
            if(prev && !(p.broken&mask) && p.ms-stamp<=750) line(f,px,prev_y,x,y,dotted);
            else f.pixel(x,y,true);
            any=true;prev=true;px=x;prev_y=y;stamp=p.ms;
        }
    }
    char a[24],b[24],legend[52];
    if(v.has(s.a)) std::snprintf(a,sizeof(a),"%.1f",v.get(s.a));else std::snprintf(a,sizeof(a),"--");
    if(v.has(s.b)) std::snprintf(b,sizeof(b),"%.1f",v.get(s.b));else std::snprintf(b,sizeof(b),"--");
    if(s.paired) std::snprintf(legend,sizeof(legend),"L ... %.12s    R ___ %.12s",a,b);
    else std::snprintf(legend,sizeof(legend),"NOW %s",a);
    fb_text(f,8,48,legend,1);
    if(!any) fb_text(f,72,122,"NO VALID HISTORY",2);
    fb_text(f,8,227,"LIVE / 60s / 2Hz",1);
}
