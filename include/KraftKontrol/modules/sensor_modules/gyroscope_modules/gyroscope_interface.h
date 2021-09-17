#ifndef GYROSCOPE_INTERFACE_H
#define GYROSCOPE_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class Gyroscope_Interface {
public:

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t gyroRate() = 0;

    /**
     * @returns reference to gyro data topic
     */
    Topic<SensorTimestamp<Vector<>>>& getGyroTopic() {return gyroTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<Vector<>>> gyroTopic_;
    
};





#endif