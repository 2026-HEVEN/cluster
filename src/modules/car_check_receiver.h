#pragma once
#include "car_check_protocol.h"
#include <cstddef>
struct CarCheckReceiver {
    car_check::Steering steering;
    car_check::Imu imu;
    car_check::Wheels wheels;
    car_check::Control control;
    car_check::Reception steering_rx,imu_rx,wheels_rx,control_rx;
    bool receive(uint32_t id,const uint8_t *data,size_t len,bool extended,bool rtr,uint32_t now);
};
