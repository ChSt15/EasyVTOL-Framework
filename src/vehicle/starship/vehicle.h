#ifndef VEHICLE_H
#define VEHICLE_H



#include "definitions.h"

#include "utils/interval_control.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "dynamics.h"
#include "flight_settings.h"


#define VEHICLE_LOOP_RATE 4000


class Vehicle: public Guidance, public Navigation, public Control, public Dynamics {
public:

    void vehicleThread();
    void vehicleInit();

private:

    IntervalControl interval = IntervalControl(VEHICLE_LOOP_RATE);

    bool _vehicleInitialized = false;

    FLIGHT_MODE _flightMode = FLIGHT_MODE::FAILSAFE;
    FLIGHT_PROFILE _flightProfile = FLIGHT_PROFILE::HOVER;

};





#endif