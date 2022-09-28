#ifndef NAVIGATION_EXTENDEDKALMAN_H
#define NAVIGATION_EXTENDEDKALMAN_H



#include "navigation_abstract.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"
#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"
#include "KraftKontrol/utils/low_pass_filter.h"

#include "lib/MathHelperLibrary/FML.h"



/**
 * @brief   First implements an extended kalman filter for attitude estimation using 
 *          gyroscope, accelerometer and magnetometer and then a kalman filter for position 
 *          estimation using GPS, Barometer and accelerometer sensors.
 * 
 */
class NavigationExtendedKalman: public Navigation_Abstract, public Task_Threading{
private:

    //Subscriber for gyro with fifo function
    Simple_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> gyroSub_;
    //Subscriber for accel with fifo function
    Simple_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> accelSub_;
    //Subscriber for mag with fifo function
    Simple_Subscriber<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> magSub_;
    //Subscriber for baro with fifo function
    Simple_Subscriber<DataTimestamped<float>> baroSub_;
    //Subscriber for gnssdata with fifo function
    Simple_Subscriber<DataTimestamped<GNSSData>> gnssSub_;


    //Newest sensor data
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> gyroData_;
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> accelData_;
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> magData_;
    DataTimestamped<float> baroData_;
    DataTimestamped<GNSSData> gnssData_;


    //Last loop sensor data
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> lastGyroData_;
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> lastAccelData_;
    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> lastMagData_;
    DataTimestamped<float> lastBaroData_;
    DataTimestamped<GNSSData> lastGnssData_;

    //Sea level pressure
    float sealevelPressure_ = 1015e2;


    //Below are the state estimation matricies.

    /// @brief state space vector for position.
    FML::Matrix_F<6, 1> positionX_;
    /// @brief state transition model matrix for position.
    FML::Matrix_F<6, 6> positionFk_;
    /// @brief control input model using accelerometer for position.
    FML::Matrix_F<6, 3> positionBk_;
    
    /// @brief covariance of position process noise.
    FML::Matrix_F<6, 6> positionQk_;

    //Below are the sensor matricies.

    /// @brief Barometer observation model for position.
    FML::Matrix_F<1, 6> baroHk_;
    /// @brief GNSS/GPS observation model for position.
    FML::Matrix_F<4, 6> gnssHk_;


    //Below is from a quick import of attitude extended kalman filter 
    // The system state x = [q, w_b, x_g]^T
	FML::Matrix_F<7, 1> attX_;
	// The system state covariance
	FML::Matrix_F<7, 7> attP_;
	// Process noise covariance matrix
	FML::Matrix_F<7, 7> U;



public:

    NavigationExtendedKalman();

    /**
     * Sets the gyroscope to be used.
     * @param gyro Gyroscope to use.
     */
    void setGyroscopeInput(const Gyroscope_Abstract& gyro);

    /**
     * Removes the gyroscope input.
     */
    void removeGyroscopeInput();

    /**
     * Sets the accelerometer to be used.
     * @param accel Accelerometer to use.
     */
    void setAccelerometerInput(const Accelerometer_Abstract& accel);

    /**
     * Removes the accelerometer input.
     */
    void removeAccelerometerInput();

    /**
     * Sets the magnetometer to be used.
     * @param mag Magnetometer to use.
     */
    void setMagnetometerInput(const Magnetometer_Abstract& mag);

    /**
     * Removes the magnetometer input.
     */
    void removeMagnetometerInput();

    /**
     * Sets the barometer to be used.
     * @param baro Barometer to use.
     */
    void setBarometerInput(const Barometer_Abstract& baro);

    /**
     * Removes the barometer input.
     */
    void removeBarometerInput();

    /**
     * Sets the GNSS to be used.
     * @param gnss GNSS to use.
     */
    void setGNSSInput(const GNSS_Abstract& gnss);

    /**
     * Removes the GNSS input.
     */
    void removeGNSSInput();

    /**
     * Initialises module. Is automatically called by scheduler.
     */
    void init() override;

    /**
     * Gives module time to do calculations and publish values. Is automatically called by scheduler.
     */
    void thread() override;


};




#endif