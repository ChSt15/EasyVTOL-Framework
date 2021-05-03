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

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "modules/module_abstract.h"

#include "navigation_interface.h"

#include "modules/sensor_modules/gyroscope_modules/gyroscope_interface.h"
#include "modules/sensor_modules/accelerometer_modules/accelerometer_interface.h"
#include "modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"
#include "modules/sensor_modules/barometer_modules/barometer_interface.h"

#include "utils/high_pass_filter.h"
#include "utils/low_pass_filter.h"

#include "data_containers/kinematic_data.h"



class NavigationComplementary: public Navigation_Interface, public Task_Abstract {
public:

    /**
     * Creates a Navigation module using complementary filters.
     * 
     * @param gyro module to use.
     * @param accel module to use.
     * @param mag module to use.
     * @param baro module to use.
     */
    NavigationComplementary(Gyroscope_Interface* gyro, Accelerometer_Interface* accel, Magnetometer_Interface* mag, Barometer_Interface* baro) : Task_Abstract(8000, eTaskPriority_t::eTaskPriority_VeryHigh, true) {
        gyro_ = gyro;
        accel_ = accel;
        mag_ = mag;
        baro_ = baro;
    }

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     * If only the kinematic parameters are needed
     * then use getKinematicData().
     *
     * @return navigation paramenters.
     */
    virtual NavigationData getNavigationData() {return navigationData_;};

    /**
     * Returns a pointer to a struct containing all 
     * the vehicles current kinematic parameters.
     * 
     * This is usefull for data linking instead of 
     * always having to pass data manually.
     *
     * @return kinematic parameter pointer.
     */
    virtual NavigationData* getNavigationDataPointer() {return &navigationData_;};


    /**
     * Returns the current position accuracy in meters.
     * Returns -1.0 if unsupported.
     * 
     * @return float.
     */
    //virtual float getPositionAccuracy() {return -1.0f;}

    /**
     * Returns the current attitude accuracy in radians.
     * Returns -1.0 if unsupported.
     * 
     * @return float.
     */
    //virtual float getAttitudeAccuracy() {return -1.0f;}


private:

    //Gyro that will be used by Navigation module
    Gyroscope_Interface* gyro_;
    //Accelerometer that will be used by Navigation module
    Accelerometer_Interface* accel_;
    //Magnetometer that will be used by Navigation module
    Magnetometer_Interface* mag_;
    //Barometer that will be used by Navigation module
    Barometer_Interface* baro_;

    //Storage container for navigationData
    NavigationData navigationData_;

    //Filter data
    HighPassFilter<Vector> _gyroHPF = HighPassFilter<Vector>(0.001);

    uint32_t _lastGyroTimestamp = 0;
    uint32_t _lastAccelTimestamp = 0;
    uint32_t _lastMagTimestamp = 0;

    uint32_t _lastBaroTimestamp = 0;

    uint32_t _lastLoopTimestamp = 0;

    Vector _lastGyroValue = 0;

    float _lastHeightValue = 0;
    //Vector _gyroOffset = 0;

    Vector _magOffset = Vector(-40.24, 56.43, -42.97);
    Vector _magScale = Vector(1.01, 1.03, 0.96);

    Vector _accelBias = 0;//Vector(-0.085,-0.07,0.105);
    Vector _accelScale = 1;//Vector(0,0,1.00765f);

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

    bool _baroInitialized = false;

    /**
     * Calculates height from current pressure and pressure at sea level for reference.
     * 
     * @param values pressure and refPressure.
     * @return float.
     */
    float _getHeightFromPressure(const float &pressure, const float &refPressure = 1000) {
        return ((float)-44330.77)*(pow(((float)pressure/(float)refPressure), 0.190263) - (float)1);
    }


};




#endif