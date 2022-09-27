#ifndef GNSS_ABSTRACT_H
#define GNSS_ABSTRACT_H



#include "stdint.h"

#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"

#include "lib/MathHelperLibrary/vector_math.h"




struct GNSSData {

    WorldPosition position;

    /// @brief in North west up coordinates
    Vector<> velocity;

    Vector<> positionError = 100000;
    Vector<> velocityError = 100000;

    int64_t gnssTOW = 0;

    bool lockValid = false;

};



class GNSS_Abstract {
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