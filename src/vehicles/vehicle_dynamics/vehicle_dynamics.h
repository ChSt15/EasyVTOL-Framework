#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class VehicleDynamics {
public:

    Quaternion getAttitude() {return _attitude;}


protected:

    virtual void sensorFusionThread();


    Vector _position;
    Vector _velocity;
    Vector _acceleration;

    Quaternion _attitude = Quaternion(Vector(1,1,1), 90*DEGREES);
    Vector _angularVelocity;

};





#endif