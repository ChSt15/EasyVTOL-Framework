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
#include "flight_modes.h"


#define LOOP_RATE_LIMIT 2000


class Navigation {
public:

    /**
     * Resets all kinetic Parameters to 0.
     * 
     * @param values none.
     * @return none.
     */
    void resetInertial() {_kineticData.position = Vector(0,0,0); _kineticData.velocity = Vector(0,0,0);}; //To be removed or replaced. This is only for testing Inertial navigation

    /**
     * Returns the vehicles attitude.
     *
     * @param values none.
     * @return vehicle attitude.
     */
    Quaternion getAttitude() {return _kineticData.attitude;}

    /**
     * Returns the vehicles angular rate.
     *
     * @param values none.
     * @return vehicle angular rate.
     */
    Vector getAngularRate() {return _kineticData.angularRate;}

    /**
     * Returns the vehicles position.
     *
     * @param values none.
     * @return vehicle position.
     */
    Vector getPosition() {return _kineticData.position;}

    /**
     * Returns the vehicles velocity.
     *
     * @param values none.
     * @return vehicle velocity.
     */
    Vector getVelocity() {return _kineticData.velocity;}

    /**
     * Returns the vehicles acceleration.
     *
     * @param values none.
     * @return vehicle acceleration.
     */
    Vector getAcceleration() {return _kineticData.acceleration;}

    /**
     * Returns the vehicles acceleration without gravity.
     *
     * @param values none.
     * @return vehicle acceleration without gravity.
     */
    Vector getLinearAcceleration() {return _kineticData.linearAcceleration;}

    /**
     * Returns a struct containing all kinetic parameters.
     *
     * @param values none.
     * @return Kinetic paramenters.
     */
    KineticData getNavigationKineticData() {return _kineticData;}


protected:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void navigationThread();
    void navigationInit(FLIGHT_MODE* flightModePointer);


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

    FLIGHT_MODE* _flightMode;


};




#endif