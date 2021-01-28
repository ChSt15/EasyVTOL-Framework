#ifndef NAVIGATION_H
#define NAVIGATION_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "sensors/imu.h"

#include "utils/device_status.h"
#include "utils/interval_control.h"


#define LOOP_RATE_LIMIT 2000


class Navigation {
public:

    void resetInertialTEST() {_position = Vector(0,0,0); _velocity = Vector(0,0,0);}; //To be removed or replaced. This is only for testing Inertial navigation

    Quaternion getAttitude() {return _attitude;}
    Vector getAngularRate() {return _angularRate;}

    Vector getPosition() {return _position;}
    Vector getVelocity() {return _velocity;}
    Vector getAcceleration() {return _acceleration;}
    Vector getLinearAcceleration() {return _linearAcceleration;}


protected:

    void sensorFusionThread();


private:

    Vector _position;
    Vector _velocity;
    Vector _acceleration;
    Vector _linearAcceleration;

    Quaternion _attitude = Quaternion(Vector(1,1,1), 0*DEGREES);
    Vector _angularRate;

    bool _angularRateValid = false;
    bool _attitudeValid = false;
    bool _headingValid = false;

    bool _accelerationValid = false;
    bool _velocityValid = false;
    bool _positionValid = false;

    bool _highPrecisionValid = false;

    uint32_t lastGyroTimestamp = 0;
    uint32_t lastAccelTimestamp = 0;
    uint32_t lastMagTimestamp = 0;

    Vector gyroAverage;

    bool gyroInitialized = false;
    bool accelInitialized = false;
    bool magInitialized = false;

    IntervalControl interval = IntervalControl(LOOP_RATE_LIMIT);


};




#endif