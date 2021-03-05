#ifndef NAVIGATION_COMPLEMENTARY_H
#define NAVIGATION_COMPLEMENTARY_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "Arduino.h"

#include "navigation_template.h"

#include "sensors/imu.h"

#include "utils/interval_control.h"
#include "utils/high_pass_filter.h"
#include "utils/low_pass_filter.h"

#include "data_containers/kinematic_data.h"



class NavigationComplementary: public Navigation {
public:

    NavigationComplementary() {
        //_connectThread();
    }

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns true if module is ready.
     *
     * @param values none.
     * @return bool.
     */
    bool moduleReady() {return _gyroInitialized && _accelInitialized && _magInitialized;};

    /**
     * Returns the current position accuracy.
     * 
     * @param values none.
     * @return float.
     */
    //float getPositionAccuracy() {return 50.0f;}


private:

    //Filter data
    //HighPassFilter<Vector> gyroHPF = HighPassFilter<Vector>(0.01);

    uint32_t _lastGyroTimestamp = 0;
    uint32_t _lastAccelTimestamp = 0;
    uint32_t _lastMagTimestamp = 0;

    uint32_t _lastLoopTimestamp = 0;

    //System information flagges
    bool _angularRateValid = false;
    bool _attitudeValid = false;
    bool _headingValid = false;

    bool _accelerationValid = false;
    bool _velocityValid = false;
    bool _positionValid = false;

    bool _highPrecisionValid = false;

    bool _gyroInitialized = false;
    bool _accelInitialized = false;
    bool _magInitialized = false;


};




#endif