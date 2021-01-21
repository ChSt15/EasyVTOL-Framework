#ifndef VEHICLE_KINEMATICS_H
#define VEHICLE_KINEMATICS_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class VehicleKinematics {
public:

    virtual void calcKinematics();

private:

    Quaternion _attitude;
    Vector _acceleration;
    Vector _velocity;

};





#endif