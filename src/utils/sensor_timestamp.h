#ifndef SENSOR_TIMESTAMP_H
#define SENSOR_TIMESTAMP_H



template<typename T>
struct SensorTimestamp{

    SensorTimestamp(T sensorData, uint32_t sensorTimestamp) {
        this->sensorData = sensorData;
        this->sensorTimestamp = sensorTimestamp;
    }

    T sensorData;

    uint32_t sensorTimestamp = 0;

};



#endif