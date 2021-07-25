#ifndef ACCELEROMETER_INTERFACE_H
#define ACCELEROMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class Accelerometer_Interface {
public:

    /**
     * @returns reference to accel data topic
     */
    Topic<SensorTimestamp<Vector<>>>& getAccelTopic() {return accelTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<Vector<>>> accelTopic_;
    
    
};





#endif