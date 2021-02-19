#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"



class VehicleTemplate {
public:

    void vehicleThread();
    void vehicleInit();

    bool vehicleReady();

    virtual void setVehicleMode(const FLIGHT_MODE &flightMode);
    virtual void setVehicleProfile(const FLIGHT_PROFILE &flightProfile);

    virtual FLIGHT_MODE getVehicleMode();
    virtual FLIGHT_PROFILE getVehicleProfile();

};





#endif