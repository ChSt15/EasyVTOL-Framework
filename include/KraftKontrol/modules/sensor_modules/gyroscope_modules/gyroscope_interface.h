#ifndef GYROSCOPE_INTERFACE_H
#define GYROSCOPE_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Gyroscope_Interface {
public:

    /**
     * @returns rate (in Hz) of the new sensor data
     */
    virtual uint32_t gyroRate() = 0;

    /**
     * @returns reference to gyro data topic
     */
    const Topic<DataTimestamped<Vector<>>>& getGyroTopic() const {return gyroTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<Vector<>>> gyroTopic_;
    
};





#endif