#ifndef BAROMETER_INTERFACE_H
#define BAROMETER_INTERFACE_H



#include "stdint.h"


#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class Barometer_Interface {
public:

    /**
     * @returns reference to baro data topic
     */
    const Topic<SensorTimestamp<float>>& getBaroTopic() const {return baroTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<float>> baroTopic_;

    
};





#endif