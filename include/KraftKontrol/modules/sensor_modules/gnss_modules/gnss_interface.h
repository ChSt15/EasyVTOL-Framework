#ifndef GNSS_INTERFACE_H
#define GNSS_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"




struct GNSSData {

    WorldPosition position;
    Vector<> velocity;

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
    const Topic<DataTimestamped<GNSSData>>& getGNSSTopic() const {return gnssTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<GNSSData>> gnssTopic_;
    
};





#endif