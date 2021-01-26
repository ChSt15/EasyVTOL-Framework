#ifndef VEHICLE_H
#define VEHICLE_H



#include "Arduino.h"

#include "vehicle_dynamics/general_dynamics/general_dynamics.h"
#include "vehicle_control/quadcopter_control/quadcopter_control.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class Vehicle: public GeneralDynamics, public QuadcopterControl {
public:

    void vehicleThread();

private:

    
};





#endif