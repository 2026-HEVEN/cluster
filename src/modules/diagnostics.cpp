#include "diagnostics.h"
#include <algorithm>
#include <cstdio>

void diagnostic_graph(FrameBuffer&,CheckUi&,const DiagnosticHistory&,const TelemetryValues&,uint32_t);
bool check_graph_for_row(CheckPage page,int row,GraphKind &kind) {
    if(page==CheckPage::Motor) {
        if(row==1) {kind=GraphKind::MotorV;return true;}
        if(row==2) {kind=GraphKind::Bus;return true;}
        if(row==3) {kind=GraphKind::Phase;return true;}
    } else if(page==CheckPage::Power) {
        if(row==2) {kind=GraphKind::BmsV;return true;}
        if(row==5) {kind=GraphKind::EmHv;return true;}
        if(row==6) {kind=GraphKind::EmLv;return true;}
        if(row==7) {kind=GraphKind::EmA;return true;}
    } else if(page==CheckPage::Vcu) {
        if(row==3) {kind=GraphKind::Throttle;return true;}
        if(row==4) {kind=GraphKind::Wss;return true;}
    } else if(page==CheckPage::Gps && row==0) {kind=GraphKind::Rtk;return true;}
    return false;
}
void CheckUi::open_graph(GraphKind kind) {
    graph_origin=page; page=CheckPage::Graph; graph=kind; graph_min=graph_max=0; graph_range_set=false;
}
void CheckUi::tap(int x,int y,bool warning) {
    if(page!=CheckPage::Home && x>=248 && y<36) {
        if(page==CheckPage::Graph) page=graph_origin;
        else if(page==CheckPage::Events) page=CheckPage::Links;
        else if(page==CheckPage::Links) page=links_origin;
        else if(page==CheckPage::Sensors || page==CheckPage::Control) page=CheckPage::Vcu;
        else if(page==CheckPage::Reasons) page=CheckPage::Control;
        else if(page==CheckPage::Menu || page==CheckPage::Warning) home();
        else page=CheckPage::Menu;
        list_page=0; return;
    }
    if(page==CheckPage::Home) {
        if(x<270 && y<108) open_graph(GraphKind::Wss);
        else page=warning?CheckPage::Warning:CheckPage::Menu;
        return;
    }
    if(page==CheckPage::Graph) return;
    if(page==CheckPage::Warning) {
        if(y>=210) { if(x>=200) page=CheckPage::Menu; else ++warning_page; }
        return;
    }
    if(page==CheckPage::Links || page==CheckPage::Events || page==CheckPage::Reasons) {
        if(y>=218) {
            if(page==CheckPage::Links && x>=160) { page=CheckPage::Events; list_page=0; }
            else ++list_page;
        }
        return;
    }
    if(y>=220) { links_origin=page; page=CheckPage::Links; list_page=0; return; }
    if(page==CheckPage::Menu && y>=40) {
        page=y<130?(x<160?CheckPage::Motor:CheckPage::Power):(x<160?CheckPage::Vcu:CheckPage::Gps);
    } else if(page==CheckPage::Motor && y>=62) {
        GraphKind kind;
        if(check_graph_for_row(page,(y-62)/17,kind)) open_graph(kind);
    } else if(y>=43) {
        int r=(y-43)/14;
        GraphKind kind;
        if(check_graph_for_row(page,r,kind)) {open_graph(kind);return;}
        if(page==CheckPage::Vcu) {
            if(r>=5 && r<=7) page=CheckPage::Control;
            if(r>=8 && r<=10) page=CheckPage::Sensors;
        } else if(page==CheckPage::Control && r==11) { page=CheckPage::Reasons; list_page=0; }
    }
}
namespace {
void button(FrameBuffer &f,int x,int y,int w,const char *s) {
    fb_rect(f,x,y,w,30,false,true); fb_text(f,x+7,y+8,s,2);
}
void header(FrameBuffer &f,const char *s) {
    fb_text(f,8,10,s,2); button(f,248,2,70,"BACK"); fb_hline(f,0,37,320,true);
}
void footer(FrameBuffer &f,const char *s="LINKS / HISTORY >") {
    fb_hline(f,0,218,320,true); fb_text(f,8,227,s,1);
}
void rows(FrameBuffer &f,const char data[12][2][25],CheckPage page) {
    for(int i=0;i<12;++i) {
        GraphKind kind;
        if(check_graph_for_row(page,i,kind)) fb_text(f,1,45+i*14,"*",1);
        fb_text(f,8,45+i*14,data[i][0],1); fb_text(f,164,45+i*14,data[i][1],1);
    }
}
void uptime(char *out,size_t n,uint32_t ms) {
    std::snprintf(out,n,"%02lu:%02lu:%02lu",static_cast<unsigned long>(ms/3600000),
        static_cast<unsigned long>(ms/60000%60),static_cast<unsigned long>(ms/1000%60));
}
void stats(FrameBuffer &f,const DiagnosticHistory &h,LinkId id,int y,uint32_t now) {
    const auto &s=h.connection(id); char t[20],b[64],drops[12],faults[12]; uptime(t,sizeof(t),s.live?now-s.since:0);
    std::snprintf(drops,sizeof(drops),s.drops>999?">999":"%lu",static_cast<unsigned long>(s.drops));
    std::snprintf(faults,sizeof(faults),s.faults>999?">999":"%lu",static_cast<unsigned long>(s.faults));
    std::snprintf(b,sizeof(b),"%.9s %.4s %.10s D%.4s F%.4s",diagnostic_link_name(id),
        s.live?"LIVE":s.seen?"LOST":"WAIT",t,drops,faults);
    fb_text(f,8,y,b,1);
}
}
void check_home_nav(FrameBuffer&,bool) {}
void check_draw(FrameBuffer &f,CheckUi &ui,const CheckSnapshot &d,const DiagnosticHistory &h,
                const TelemetryValues &v,uint32_t now) {
    if(ui.page==CheckPage::Graph) { diagnostic_graph(f,ui,h,v,now);return; }
    if(ui.page==CheckPage::Menu) {
        header(f,"CAR CHECK"); fb_vline(f,160,38,180,true);fb_hline(f,0,130,320,true);
        const char *names[]={"MOTOR","POWER","VCU/SENSOR","GPS/RTK"};
        for(int i=0;i<4;++i) {
            int x=i%2*160+8,y=49+i/2*90;
            fb_text(f,x,y,names[i],2);fb_text(f,x,y+28,d.summaries[i][0],1);fb_text(f,x,y+45,d.summaries[i][1],1);
        } footer(f);
    } else if(ui.page==CheckPage::Motor) {
        header(f,"MOTOR");fb_text(f,112,45,"LEFT ...",1);fb_text(f,216,45,"RIGHT ___",1);
        for(int i=0;i<9;++i) for(int j=0;j<3;++j) fb_text(f,j==0?8:j==1?112:216,64+i*17,d.motor[i][j],1);
        for(int i=0;i<9;++i) {GraphKind kind;if(check_graph_for_row(ui.page,i,kind)) fb_text(f,1,64+i*17,"*",1);}
        footer(f);
    } else if(ui.page==CheckPage::Power) {header(f,"POWER");rows(f,d.power,ui.page);footer(f);}
    else if(ui.page==CheckPage::Vcu) {header(f,"VCU / SENSOR");rows(f,d.vcu,ui.page);footer(f);}
    else if(ui.page==CheckPage::Gps) {header(f,"GPS / RTK");rows(f,d.gps,ui.page);footer(f);}
    else if(ui.page==CheckPage::Sensors) {header(f,"SENSORS");rows(f,d.sensors,ui.page);footer(f);}
    else if(ui.page==CheckPage::Control) {header(f,"CONTROL");rows(f,d.control,ui.page);footer(f);}
    else if(ui.page==CheckPage::Links) {
        header(f,"CONNECTIONS");unsigned pages=(LINKS+6)/7;ui.list_page%=pages;
        fb_text(f,8,44,"STATE / CONTINUOUS UP / DROPS / FAULTS",1);
        for(unsigned i=0;i<7;++i) {unsigned n=ui.list_page*7+i;if(n<LINKS) stats(f,h,static_cast<LinkId>(n),64+i*21,now);}
        footer(f,"NEXT LINKS                 EVENTS >");
    } else if(ui.page==CheckPage::Events) {
        header(f,"EVENT HISTORY");unsigned pages=std::max<size_t>(1,(h.event_count()+5)/6);ui.list_page%=pages;
        char b[56];std::snprintf(b,sizeof(b),"THIS BOOT / %u/%u / overwritten %lu",ui.list_page+1,pages,static_cast<unsigned long>(h.event_overwrites()));fb_text(f,8,43,b,1);
        for(unsigned i=0;i<6;++i) {
            unsigned n=ui.list_page*6+i;if(n>=h.event_count()) break;
            const auto &e=h.event(n);char t[20];uptime(t,sizeof(t),e.ms);
            const char *kind=e.kind==EventKind::Connected?"UP":e.kind==EventKind::Lost?"LOST":e.kind==EventKind::Restored?"RESTORED":e.kind==EventKind::FaultOn?"FAULT":"CLEARED";
            std::snprintf(b,sizeof(b),"%s %s %s",t,diagnostic_link_name(e.node),kind);fb_text(f,8,58+i*26,b,1);
            if(e.kind==EventKind::FaultOn||e.kind==EventKind::FaultOff) fb_text(f,20,69+i*26,diagnostic_fault_name(e.bit),1);
        } if(!h.event_count()) fb_text(f,8,85,"NO EVENTS",2);footer(f,"NEXT >");
    } else if(ui.page==CheckPage::Reasons) {
        header(f,"BLOCK REASONS");ui.list_page%=2;
        if(!d.control_valid) fb_text(f,8,48,"CONTROL -- / STALE / UNKNOWN",1);
        else for(unsigned i=0;i<8;++i){unsigned n=ui.list_page*8+i;if(n<15) fb_text(f,8,48+i*20,d.reasons[n],1);}
        footer(f,"NEXT >");
    } else if(ui.page==CheckPage::Warning) {
        header(f,"WARNING");unsigned pages=std::max(1,(d.warning_count+7)/8);ui.warning_page%=pages;
        for(int i=0;i<8;++i){int n=ui.warning_page*8+i;if(n<d.warning_count) fb_text(f,8,46+i*20,d.warnings[n],2);}
        char b[24];std::snprintf(b,sizeof(b),"%u/%u NEXT",ui.warning_page+1,pages);fb_text(f,8,222,b,1);button(f,200,210,118,"CHECK");
    }
}
