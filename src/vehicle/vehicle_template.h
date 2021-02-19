#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"



class VehicleTemplate {
public:

    virtual void vehicleThread();
    virtual void vehicleInit();

    virtual bool vehicleReady();

    virtual void setVehicleMode(FLIGHT_MODE flightMode);
    virtual void setVehicleProfile(FLIGHT_PROFILE flightProfile);

};





#endif