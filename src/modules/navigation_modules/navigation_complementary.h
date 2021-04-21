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

#include "modules/sensor_modules/mpu9250_driver.h"
#include "modules/sensor_modules/bme280_driver.h"

#include "utils/high_pass_filter.h"
#include "utils/low_pass_filter.h"

#include "data_containers/kinematic_data.h"



class NavigationComplementary: public Navigation {
public:

    NavigationComplementary() {
        
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
     * Sets the vehicle data input pointer.
     * 
     * Allows the control module to automatically retrieve its needed
     * data from the pointer.
     * 
     * This must only be called once.
     * 
     * returns false if failed from null pointer input.
     *
     * @param values vehicleDataPointer.
     * @return bool.
     */
    bool linkVehicleDataPointer(VehicleData *vehicleDataPointer) {
        if (vehicleDataPointer == nullptr) return false;
        _vehicleData = vehicleDataPointer;
        return true;
    };

    /**
     * Returns the current position accuracy.
     * 
     * @param values none.
     * @return float.
     */
    //float getPositionAccuracy() {return 50.0f;}


private:

    //Vehicle data pointer
    VehicleData* _vehicleData = nullptr;

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