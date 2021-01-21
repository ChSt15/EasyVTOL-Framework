#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class VehicleDynamics {
public:

    virtual void sensorFusionThread();

private:

    Vector _position;
    Vector _velocity;
    Vector _acceleration;

    Quaternion _attitude;
    Vector _angularVelocity;

};





#endif