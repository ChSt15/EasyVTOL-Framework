#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"



class VehicleControl {
public:

    virtual void controlThread();

private:

    Vector _setPosition;
    Vector _setVelocity;
    Vector _setAcceleration;

    Quaternion _setAttitude;
    Vector _setAngularVelocity;

};




#endif