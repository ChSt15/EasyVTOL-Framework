#ifndef GNSS_INTERFACE_H
#define GNSS_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"




struct GNSSData {

    SensorTimestamp<WorldPosition> positionValueTimestamp;
    SensorTimestamp<Vector<>> velocityValueTimestamp;

    float positionError = 100000;
    float altitudeError = 100000;
    float velocityError = 100000;

    bool lockValid = false;

};



class GNSS_Interface {
public:

    /**
     * @returns reference to gnss data topic
     */
    Topic<GNSSData>& getGNSSTopic() {return gnssTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<GNSSData> gnssTopic_;
    
};





#endif