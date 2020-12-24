#ifndef IMU_H
#define IMU_H



#include "Arduino.h"

#include "SparkFunBME280.h"
#include "vector_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define BME_RATE 50
#define MEASUREMENT_TIMEOUT_US 200000L



namespace AirData {

    extern CircularBuffer <float, 50> pressureFifo;
    extern CircularBuffer <float, 50> humidityFifo;
    extern CircularBuffer <float, 50> temperatureFifo;
    extern CircularBuffer <uint32_t, 50> timestampFifo;

    uint16_t getDataCount();

    void imuThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif