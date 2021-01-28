#ifndef NAVIGATION_H
#define NAVIGATION_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "Arduino.h"

#include "sensors/imu.h"

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

    //System inertial information
    Vector _position;
    Vector _velocity;
    Vector _acceleration;
    Vector _linearAcceleration;

    Quaternion _attitude = Quaternion(Vector(1,1,1), 0*DEGREES);
    Vector _angularRate;

    //Filter data
    Vector gyroAverage;

    uint32_t lastGyroTimestamp = 0;
    uint32_t lastAccelTimestamp = 0;
    uint32_t lastMagTimestamp = 0;

    //System information flagges
    bool _angularRateValid = false;
    bool _attitudeValid = false;
    bool _headingValid = false;

    bool _accelerationValid = false;
    bool _velocityValid = false;
    bool _positionValid = false;

    bool _highPrecisionValid = false;

    bool gyroInitialized = false;
    bool accelInitialized = false;
    bool magInitialized = false;

    //Loop interval control
    IntervalControl interval = IntervalControl(LOOP_RATE_LIMIT);


};




#endif