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

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "navigation_interface.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_interface.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_interface.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_interface.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_interface.h"

#include "KraftKontrol/utils/high_pass_filter.h"
#include "KraftKontrol/utils/low_pass_filter.h"
#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/utils/value_error.h"
#include "KraftKontrol/utils/system_time.h"

#include "KraftKontrol/data_containers/kinematic_data.h"

#include "KraftKontrol/utils/topic_subscribers.h"



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
    NavigationComplementaryFilter(Gyroscope_Interface& gyro, Accelerometer_Interface& accel, Magnetometer_Interface* mag = nullptr, Barometer_Interface* baro = nullptr, GNSS_Interface* gnss = nullptr) : Task_Abstract(8000, eTaskPriority_t::eTaskPriority_VeryHigh, true) {
        gyroSub_.subscribe(gyro.getGyroTopic());
        accelSub_.subscribe(accel.getAccelTopic());
        if (mag != nullptr) magSub_.subscribe(mag->getMagTopic());
        if (baro != nullptr) baroSub_.subscribe(baro->getBaroTopic());
        if (gnss != nullptr) gnssSub_.subscribe(gnss->getGNSSTopic());
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
     * @returns magnetic vector
     */
    //Vector<> getMag() {return magVec_;}


private:

    //Subscriber for gyro with fifo function
    Buffer_Subscriber<SensorTimestamp<Vector<>>, 100> gyroSub_;
    //Subscriber for accel with fifo function
    Buffer_Subscriber<SensorTimestamp<Vector<>>, 100> accelSub_;
    //Subscriber for mag with fifo function
    Buffer_Subscriber<SensorTimestamp<Vector<>>, 20> magSub_;
    //Subscriber for baro with fifo function
    Buffer_Subscriber<SensorTimestamp<float>, 10> baroSub_;
    //Subscriber for gnssdata with fifo function
    Buffer_Subscriber<GNSSData, 10> gnssSub_;

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
    int64_t lastGyroTimestamp_ = 0;
    int64_t lastAccelTimestamp_ = 0;
    int64_t lastMagTimestamp_ = 0;
    int64_t lastBaroTimestamp_ = 0;

    int64_t lastLoopTimestamp_ = 0;

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