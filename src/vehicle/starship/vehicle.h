#ifndef VEHICLE_H
#define VEHICLE_H



#include "definitions.h"

#include "vehicle/vehicle_template.h"

#include "utils/interval_control.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "dynamics.h"
#include "output_control.h"

#include "vehicle/flight_modes.h"


#define VEHICLE_LOOP_RATE 4000


class Vehicle: public VehicleTemplate, public Guidance, public Navigation, public Control, public Dynamics, public OutputControl {
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