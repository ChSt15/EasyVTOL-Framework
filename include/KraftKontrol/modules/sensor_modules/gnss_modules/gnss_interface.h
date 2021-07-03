#ifndef GNSS_INTERFACE_H
#define GNSS_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"



class GNSS_Interface;


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
     * Returns rate (in Hz) of the thread
     *
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t positionRate() = 0;

    /**
     * @returns true if lock is valid
     */
    virtual bool getGNSSLockValid() = 0;

    /**
     * @returns reference to gnss data topic
     */
    Topic<GNSSData>& getGNSSTopic() {return gnssTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<GNSSData> gnssTopic_;
    
};





#endif