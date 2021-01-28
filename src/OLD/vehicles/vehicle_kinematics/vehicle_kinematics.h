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

    Vector _position;
    Vector _velocity;
    Vector _acceleration;

    Quaternion _attitude;
    Vector _angularVelocity;

};





#endif