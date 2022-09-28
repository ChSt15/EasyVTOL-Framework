#ifndef DATA_TIMESTAMPED_H
#define DATA_TIMESTAMPED_H



//#include "KraftKontrol/utils/system_time.h"

#include "stdint.h"

#include "time_definitions.h"


/**
 * Designed to take data and save it directly with a timestamp.
 */
template<typename T>
struct DataTimestamped {

    DataTimestamped() {
        timestamp = 0;
    }

    /**
     * @param data Sensordata to take.
     * @param timestamp Time at which sensordata was created. Defaults to current system time via NOW().
     */
    DataTimestamped(const T& data, int64_t timestamp = NOW()) {
        this->data = data;
        this->timestamp = timestamp;
    }

    /**
     * @param data Sensordata to take.
     * @param timestamp Time at which sensordata was created. Defaults to current system time via NOW().
     */
    DataTimestamped(const T&& data, int64_t timestamp = NOW()) {
        this->data = data;
        this->timestamp = timestamp;
    }

    //Data that is timestamped
    T data;
    //Timestamp for Data in Nanoseconds
    int64_t timestamp;

};



#endif