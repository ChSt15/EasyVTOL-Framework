#ifndef ACCELEROMETER_INTERFACE_H
#define ACCELEROMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class Accelerometer_Interface {
public:

    /**
     * @returns rate (in Hz) of the thread
     */
    virtual uint32_t loopRate() = 0;

    /**
     * @returns rate (in Hz) of the new sensor data
     */
    virtual uint32_t accelRate() = 0;

    /**
     * @returns reference to accel data topic
     */
    Topic<SensorTimestamp<Vector<>>>& getAccelTopic() {return accelTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<Vector<>>> accelTopic_;
    
    
};





#endif