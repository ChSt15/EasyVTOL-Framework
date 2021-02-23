#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H


#include "definitions.h"

#include "data_containers/dynamic_data.h"
#include "data_containers/kinetic_data.h"






/**
 * Struct containing all vehicle global data.
 */
struct VehicleData {

    //Contains current vehicle movement.
    KinematicData vehicleKinematics;
    //Contains current forces exerted on the vehicle.
    DynamicData vehicleDynamics;

    //Contains the vehicles current mode. At bootup its set to FAILSAFE.
    //VEHICLE_MODE vehicleMode = VEHICLE_MODE::MODE_FAILSAFE;
    
};



#endif