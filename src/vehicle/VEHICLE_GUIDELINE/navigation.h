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
#include "utils/high_pass_filter.h"
#include "utils/low_pass_filter.h"

#include "vehicle/kinetic_data.h"


#define LOOP_RATE_LIMIT 2000


class Navigation {
public:

    void resetInertialTEST() {_kineticData.position = Vector(0,0,0); _kineticData.velocity = Vector(0,0,0);}; //To be removed or replaced. This is only for testing Inertial navigation

    Quaternion getAttitude() {return _kineticData.attitude;}
    Vector getAngularRate() {return _kineticData.angularRate;}

    Vector getPosition() {return _kineticData.position;}
    Vector getVelocity() {return _kineticData.velocity;}
    Vector getAcceleration() {return _kineticData.acceleration;}
    Vector getLinearAcceleration() {return _kineticData.linearAcceleration;}

    KineticData getKineticData() {return _kineticData;}


protected:

    void navigationThread();


private:

    //System inertial information
    KineticData _kineticData;

    //Filter data
    //HighPassFilter<Vector> gyroHPF = HighPassFilter<Vector>(0.01);

    uint32_t _lastGyroTimestamp = 0;
    uint32_t _lastAccelTimestamp = 0;
    uint32_t _lastMagTimestamp = 0;

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
    IntervalControl _interval = IntervalControl(LOOP_RATE_LIMIT);


};




#endif