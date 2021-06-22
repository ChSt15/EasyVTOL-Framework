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
#include "modules/sensor_modules/gnss_modules/gnss_interface.h"

#include "utils/high_pass_filter.h"
#include "utils/low_pass_filter.h"
#include "utils/buffer.h"
#include "utils/value_error.h"
#include "utils/system_time.h"

#include "data_containers/kinematic_data.h"



class NavigationComplementaryFilter: public Navigation_Interface, public Task_Abstract {
public:

    /**
     * Creates a Navigation module using complementary filters.
     * 
     * @param gyro module to use.
     * @param accel module to use.
     * @param mag module to use.
     * @param baro module to use.
     * @param gnss module to use.
     */
    NavigationComplementaryFilter(Gyroscope_Interface* gyro, Accelerometer_Interface* accel, Magnetometer_Interface* mag = nullptr, Barometer_Interface* baro = nullptr, GNSS_Interface* gnss = nullptr) : Task_Abstract(8000, eTaskPriority_t::eTaskPriority_VeryHigh, true) {
        gyro_ = gyro;
        accel_ = accel;
        mag_ = mag;
        baro_ = baro;
        gnss_ = gnss;
    }

    /**
     * This is where all calculations are done.
     */
    void thread();

    /**
     * Initialisation of module.
     */
    void init();

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     * If only the kinematic parameters are needed
     * then use getKinematicData().
     *
     * @return navigation paramenters.
     */
    NavigationData getNavigationData() {return navigationData_;};

    /**
     * Returns a pointer to a struct containing all 
     * the vehicles current kinematic parameters.
     * 
     * This is usefull for data linking instead of 
     * always having to pass data manually.
     *
     * @return kinematic parameter pointer.
     */
    NavigationData* getNavigationDataPointer() {return &navigationData_;};

    /**
     * Sets the home position.
     * All position data will be in refernce to this home position.
     *
     * @param homePosition Is the position to be used as home.
     */
    void setHome(WorldPosition homePosition) {

        navigationData_.homePosition = homePosition;
        navigationData_.position = Vector<>(0);

    }


    /**
     * @returns magnetic vector
     */
    Vector<> getMag() {return magVec_;}


private:

    /**
     * Predicts system state at given time.
     * @param systemState Current system state to predict.
     * @param time Absolute time of where system should be. In nanoseconds.
     * @returns predicted state.
     */
    KinematicData predictState(const KinematicData &systemState, const int64_t &time);

    //Gyro that will be used by Navigation module
    Gyroscope_Interface* gyro_ = nullptr;
    //Accelerometer that will be used by Navigation module
    Accelerometer_Interface* accel_ = nullptr;
    //Magnetometer that will be used by Navigation module
    Magnetometer_Interface* mag_ = nullptr;
    //Barometer that will be used by Navigation module
    Barometer_Interface* baro_ = nullptr;
    //GNSS module that will be used by Navigation module.
    GNSS_Interface* gnss_ = nullptr;
    

    //Storage container for navigationData
    NavigationData navigationData_;

    //Filter data
    LowPassFilter<Vector<>> gyroLPF_ = LowPassFilter<Vector<>>(0.01);
    LowPassFilter<Vector<>> accelBiasLPF_ = LowPassFilter<Vector<>>(0.2);
    LowPassFilter<Vector<>> accelLPF_ = LowPassFilter<Vector<>>(3000);

    //Buffers
    Buffer<float, 10> gyroXBuffer_;
    Buffer<float, 10> gyroYBuffer_;
    Buffer<float, 10> gyroZBuffer_;

    Buffer<float, 50> accelXBuffer_;
    Buffer<float, 50> accelYBuffer_;
    Buffer<float, 50> accelZBuffer_;

    Buffer<float, 5> gnssPositionXBuffer_;
    Buffer<float, 5> gnssPositionYBuffer_;
    Buffer<float, 5> gnssPositionZBuffer_;

    Buffer<float, 3> gnssVelocityXBuffer_;
    Buffer<float, 3> gnssVelocityYBuffer_;
    Buffer<float, 3> gnssVelocityZBuffer_;

    Buffer<float, 50> baroHeightBuffer_;
    Buffer<float, 50> baroVelBuffer_;

    //timestamps
    uint32_t lastGyroTimestamp_ = 0;
    uint32_t lastAccelTimestamp_ = 0;
    uint32_t lastMagTimestamp_ = 0;
    uint32_t lastBaroTimestamp_ = 0;

    uint32_t lastLoopTimestamp_ = 0;

    Vector<> lastGyroValue_ = 0;
    ValueError<Vector<>> lastAngularRateValue_ = ValueError<Vector<>>(0, 0);
    float _lastHeightValue = 0;
    //Vector<> _gyroOffset = 0;

    float baroPressure_ = 0;
    float sealevelPressure_ = 100e3;
    bool seaLevelPressureCorrected_ = false;


    Vector<> magVec_ = 0;

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
     * @param pressure Air pressure at current position.
     * @param refPressure Air pressure at sea level.
     * @returns altitude above reference pressure (sea level)
     */
    float getHeightFromPressure(const float &pressure, const float &refPressure) {
        return ((float)-44330.77)*(pow(((float)pressure/(float)refPressure), 0.190263) - (float)1);
    }

    /**
     * Calculates height from current pressure and pressure at sea level for reference.
     * 
     * @param pressure Air pressure at current position.
     * @param height Current height.
     * @returns calclulated sealevel pressure.
     */
    float getSealevelPressureFromHeight(const float &pressure, const float &height) {
        return pressure/pow(height/((float)-44330.77) + 1, 5.25588f);
    }


};




#endif