#ifndef AIR_DATA_H
#define AIR_DATA_H



#include "Arduino.h"

#include "SparkFunBME280.h"
#include "vector_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define BME_RATE 20



namespace AirData {

    extern CircularBuffer <float, 50> pressureFifo;
    extern CircularBuffer <float, 50> humidityFifo;
    extern CircularBuffer <float, 50> temperatureFifo;
    extern CircularBuffer <uint32_t, 50> timestampFifo;

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif