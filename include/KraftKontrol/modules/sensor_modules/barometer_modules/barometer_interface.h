#ifndef BAROMETER_INTERFACE_H
#define BAROMETER_INTERFACE_H



#include "stdint.h"


#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class Barometer_Interface {
public:

    /**
     * @returns rate (in Hz) of the thread
     */
    virtual uint32_t loopRate() = 0;

    /**
     * @returns rate (in Hz) of new sensor data
     */
    virtual uint32_t pressureRate() = 0;

    /**
     * @returns reference to baro data topic
     */
    Topic<SensorTimestamp<float>>& getBaroTopic() {return baroTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<float>> baroTopic_;

    
};





#endif