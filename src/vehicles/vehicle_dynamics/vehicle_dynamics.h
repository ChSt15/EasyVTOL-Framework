#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class VehicleDynamics {
public:

    Quaternion getAttitude() {return _attitude;}
    Vector getAngularRate() {return _angularRate;}


protected:

    virtual void sensorFusionThread();


    Vector _position;
    Vector _velocity;
    Vector _acceleration;

    Quaternion _attitude = Quaternion(Vector(1,1,1), 0*DEGREES);
    Vector _angularRate;

    bool _angularRateValid = false;
    bool _attitudeValid = false;
    bool _headingValid = false;

    bool _accelerationValid = false;
    bool _velocityValid = false;
    bool _positionValid = false;

    bool _highPrecisionValid = false;

};





#endif