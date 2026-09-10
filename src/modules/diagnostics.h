#pragma once
#include "framebuffer.h"
#include "diagnostic_history.h"
enum class CheckPage { Home, Warning, Menu, Motor, Power, Vcu, Gps,
    Sensors, Control, Reasons, Links, Events, Graph };
enum class GraphKind { Wss, Bus, Phase, MotorV, EmHv, EmLv, BmsV, EmA, Throttle, Rtk };
bool check_graph_for_row(CheckPage page,int row,GraphKind &kind);
struct CheckUi {
    CheckPage page=CheckPage::Home, graph_origin=CheckPage::Home, links_origin=CheckPage::Menu;
    GraphKind graph=GraphKind::Wss;
    unsigned warning_page=0, list_page=0;
    float graph_min=0, graph_max=0;
    bool graph_range_set=false;
    void home() { page=CheckPage::Home; }
    void open_graph(GraphKind kind);
    void tap(int x,int y,bool warning);
};
struct CheckSnapshot {
    char summaries[4][2][24]{};
    char motor[9][3][20]{};
    char power[12][2][25]{},vcu[12][2][25]{},gps[12][2][25]{};
    char sensors[12][2][25]{},control[12][2][25]{};
    char reasons[15][40]{};
    bool control_valid=false;
    const char *warnings[48]{};
    int warning_count=0;
};
struct HomeData {
    float speed=0,throttle=0,hv=0,lv=0,power_kw=0;
    bool speed_ok=false,throttle_ok=false,em_ok=false,gear_ok=false,charging=false;
    int soc=-1;
    uint8_t gear=0,lap=0,best_lap=0;
    uint32_t lap_ms=0,best_ms=0;
};
void check_draw(FrameBuffer&,CheckUi&,const CheckSnapshot&,const DiagnosticHistory&,
                const TelemetryValues&,uint32_t now);
void check_home_draw(FrameBuffer&,const HomeData&,bool warning);
void check_home_nav(FrameBuffer&,bool warning);
