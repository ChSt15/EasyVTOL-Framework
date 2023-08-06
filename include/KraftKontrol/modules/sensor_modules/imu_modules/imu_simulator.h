#ifndef IMU_SIMULATOR_H
#define IMU_SIMULATOR_H



#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/topic_subscribers.h"

#include "lib/MathHelperLibrary/FML.h"




class IMUSimulator: public Gyroscope_Abstract, public Accelerometer_Abstract, public Magnetometer_Abstract, public Module_Abstract, public Task_Threading {
public:

    IMUSimulator(Topic<DataTimestamped<FML::Quat_F>> attitudeTopic, Topic<DataTimestamped<FML::Matrix_F<3, 1>>> positionTopic, float gyroVariance = 0.005, float accelVariance = 0.07, float magVariance = 0.5): Task_Threading("IMU Simulator", eTaskPriority_t::eTaskPriority_Realtime, 1*MILLISECONDS) {
        
        attitudeSubr_.subscribe(attitudeTopic);
        positionSubr_.subscribe(positionTopic);

        gyroVariance_ = gyroVariance;
        accelVariance_ = accelVariance;
        magVariance_ = magVariance;

    }
    

    void thread() override;

    void init() override;


private:

    DataTimestamped<FML::Quat_F> lastAttitude_;
    DataTimestamped<FML::Matrix_F<3, 1>> lastPosition_;
    FML::Matrix_F<3, 1> lastVelocity_;

    //Accel in world coordinate frame
    FML::Matrix_F<3, 1> lastAccel_;

    Simple_Subscriber<DataTimestamped<FML::Quat_F>> attitudeSubr_;
    Simple_Subscriber<DataTimestamped<FML::Matrix_F<3, 1>>> positionSubr_;

    float gyroVariance_, accelVariance_, magVariance_;

    bool isInitialized_ = false;
    
    
};



#endif