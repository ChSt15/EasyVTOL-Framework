#ifndef SENSOR_TIMESTAMP_H
#define SENSOR_TIMESTAMP_H



template<typename T>
struct SensorTimestamp{

    SensorTimestamp() {
        //this->sensorData = 0;
        //this->sensorTimestamp = 0;
    }

    SensorTimestamp(T sensorData, int64_t sensorTimestamp) {
        this->sensorData = sensorData;
        this->sensorTimestamp = sensorTimestamp;
    }

    T sensorData;

    int64_t sensorTimestamp = 0;

};



#endif