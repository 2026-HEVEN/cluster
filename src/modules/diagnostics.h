#pragma once
#include "framebuffer.h"
#include "diagnostic_history.h"
enum class CheckPage { Home, Warning, Menu, Motor, Power, Vcu, Gps,
    Sensors, Control, Reasons, Links, Events, Graph };
enum class GraphKind { Wss, Bus, Phase, MotorV, BmsV, BmsA, BmsPower, Throttle, Rtk,
    LapBattery };
constexpr size_t LAP_BATTERY_SLOTS=99;
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
    char motor[10][3][20]{};
    char power[12][2][25]{},vcu[12][2][25]{},gps[12][2][25]{};
    char sensors[12][2][25]{},control[12][2][25]{};
    char reasons[15][40]{};
    bool control_valid=false;
    const char *warnings[48]{};
    int warning_count=0;
    uint16_t lap_battery_x10[LAP_BATTERY_SLOTS]{};
    bool lap_battery_valid[LAP_BATTERY_SLOTS]{};
    uint8_t lap_battery_count=0,lap_battery_active=0;
};
struct HomeData {
    float speed=0,throttle=0,hv=0;
    bool speed_ok=false,throttle_ok=false,bms_ok=false,gear_ok=false;
    bool brake_valid=false,brake_active=false;
    int soc=-1;
    uint8_t gear=0,lap=0,best_lap=0;
    uint32_t lap_ms=0,best_ms=0;
    uint16_t lap_battery_x10=0,last_lap_battery_x10=0;
    bool lap_battery_valid=false,last_lap_battery_valid=false;
};
void check_draw(FrameBuffer&,CheckUi&,const CheckSnapshot&,const DiagnosticHistory&,
                const TelemetryValues&,uint32_t now);
void check_home_draw(FrameBuffer&,const HomeData&,bool warning);
void check_home_nav(FrameBuffer&,bool warning);
