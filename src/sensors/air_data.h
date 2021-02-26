#ifndef AIR_DATA_H
#define AIR_DATA_H



#include "Arduino.h"

#include "definitions.h"

#include "SparkFunBME280.h"
#include "3d_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"



#define BME_RATE 20

#define SENSOR_MEASUREMENT_TIMEOUT_US 20000



namespace AirData {

    extern CircularBuffer <float, 50> pressureFifo;
    extern CircularBuffer <float, 50> humidityFifo;
    extern CircularBuffer <float, 50> temperatureFifo;
    extern CircularBuffer <uint32_t, 50> timestampFifo;

    void deviceThread();

    uint32_t getRate();
    uint32_t getMeasurementRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif