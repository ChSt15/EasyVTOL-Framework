#ifndef SENSOR_TIMESTAMP_H
#define SENSOR_TIMESTAMP_H



/**
 * Designed to take sensor data (Or whatever else) and save it directly with a timestamp.
 */
template<typename T>
struct SensorTimestamp{

    SensorTimestamp() {}

    /**
     * @param sensorData Sensordata to take.
     * @param sensorTimestamp Time at which sensordata was created. Defaults to current system time via NOW().
     */
    SensorTimestamp(const T& sensorData, int64_t sensorTimestamp = NOW()) {
        this->sensorData = sensorData;
        this->sensorTimestamp = sensorTimestamp;
    }

    /**
     * @param sensorData Sensordata to take.
     * @param sensorTimestamp Time at which sensordata was created. Defaults to current system time via NOW().
     */
    SensorTimestamp(const T&& sensorData, int64_t sensorTimestamp = NOW()) {
        this->sensorData = sensorData;
        this->sensorTimestamp = sensorTimestamp;
    }

    T sensorData;

    int64_t sensorTimestamp;

};



#endif