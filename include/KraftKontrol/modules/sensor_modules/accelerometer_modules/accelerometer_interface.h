#ifndef ACCELEROMETER_INTERFACE_H
#define ACCELEROMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Accelerometer_Interface {
public:

    /**
     * @returns reference to accel data topic
     */
    const Topic<DataTimestamped<Vector<>>>& getAccelTopic() const {return accelTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<Vector<>>> accelTopic_;
    
    
};





#endif