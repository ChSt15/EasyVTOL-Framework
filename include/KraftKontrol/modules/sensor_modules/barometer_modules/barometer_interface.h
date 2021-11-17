#ifndef BAROMETER_INTERFACE_H
#define BAROMETER_INTERFACE_H



#include "stdint.h"


#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Barometer_Interface {
public:

    /**
     * @returns reference to baro data topic
     */
    const Topic<DataTimestamped<float>>& getBaroTopic() const {return baroTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<float>> baroTopic_;

    
};





#endif