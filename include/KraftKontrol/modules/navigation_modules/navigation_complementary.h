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

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "navigation_abstract.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_abstract.h"

#include "KraftKontrol/utils/high_pass_filter.h"
#include "KraftKontrol/utils/low_pass_filter.h"
#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/utils/value_error.h"
#include "KraftKontrol/utils/system_time.h"
#include "lib/MathHelperLibrary/FML.h"

#include "lib/MathHelperLibrary/vector_math.h"

#include "KraftKontrol/data_containers/kinematic_data.h"

#include "KraftKontrol/utils/topic_subscribers.h"



class NavigationComplementaryFilter: public Navigation_Abstract, public Task_Threading {
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
    NavigationComplementaryFilter(Gyroscope_Abstract* gyro = nullptr, Accelerometer_Abstract* accel = nullptr, Magnetometer_Abstract* mag = nullptr, Barometer_Abstract* baro = nullptr, GNSS_Abstract* gnss = nullptr) : Task_Threading("Complementary Navigation", eTaskPriority_t::eTaskPriority_VeryHigh, SECONDS/4000) {
        if (gyro != nullptr) {
            gyroSub_.subscribe(gyro->getGyroTopic()); 
            gyroSub_.setOverwrite(true);
        }
        if (accel != nullptr) {
            accelSub_.subscribe(accel->getAccelTopic()); 
            accelSub_.setOverwrite(true);
        }
        if (mag != nullptr) {magSub_.subscribe(mag->getMagTopic()); }
        if (baro != nullptr) {baroSub_.subscribe(baro->getBaroTopic()); }
        if (gnss != nullptr) {gnssSub_.subscribe(gnss->getGNSSTopic()); }
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
    //VectorOLD<> getMag() {return magVec_;}


private:

    //Subscriber for gyro with fifo function
    Buffer_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>, 100> gyroSub_;
    //Subscriber for accel with fifo function
    Buffer_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>, 100> accelSub_;
    //Subscriber for mag with fifo function
    Buffer_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>, 20> magSub_;
    //Subscriber for baro with fifo function
    Buffer_Subscriber<DataTimestamped<float>, 10> baroSub_;
    //Subscriber for gnssdata with fifo function
    Buffer_Subscriber<DataTimestamped<GNSSData>, 10> gnssSub_;

    //Filter data
    LowPassFilter<VectorOLD<>> gyroLPF_ = LowPassFilter<VectorOLD<>>(0.01);
    LowPassFilter<VectorOLD<>> accelBiasLPF_ = LowPassFilter<VectorOLD<>>(0.2, 8000);
    LowPassFilter<VectorOLD<>> accelLPF_ = LowPassFilter<VectorOLD<>>(100);

    VectorOLD<> accelBias_ = 0;
    VectorOLD<> gravity_ = VectorOLD<>(0,0,9.81);

    //Buffers
    Buffer<float, 10> gyroXBuffer_;
    Buffer<float, 10> gyroYBuffer_;
    Buffer<float, 10> gyroZBuffer_;

    Buffer<float, 20> accelXBuffer_;
    Buffer<float, 20> accelYBuffer_;
    Buffer<float, 20> accelZBuffer_;

    Buffer<float, 5> gnssPositionXBuffer_;
    Buffer<float, 5> gnssPositionYBuffer_;
    Buffer<float, 5> gnssPositionZBuffer_;

    Buffer<float, 3> gnssVelocityXBuffer_;
    Buffer<float, 3> gnssVelocityYBuffer_;
    Buffer<float, 3> gnssVelocityZBuffer_;

    Buffer<float, 30> baroHeightBuffer_;
    Buffer<float, 30> baroVelBuffer_;

    //timestamps
    int64_t lastGyroTimestamp_ = 0;
    int64_t lastAccelTimestamp_ = 0;
    int64_t lastMagTimestamp_ = 0;
    int64_t lastBaroTimestamp_ = 0;

    int64_t lastLoopTimestamp_ = 0;

    VectorOLD<> lastGyroValue_ = 0;
    ValueError<VectorOLD<>> lastAngularRateValue_ = ValueError<VectorOLD<>>(0, 0);
    float _lastHeightValue = 0;
    //VectorOLD<> _gyroOffset = 0;

    float baroPressure_ = 0;
    float sealevelPressure_ = 100e3;
    bool seaLevelPressureCorrected_ = false;


    VectorOLD<> magVec_ = 0;

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