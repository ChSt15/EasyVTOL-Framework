#ifndef GPS_H
#define GPS_H

#include "Arduino.h"

#include "definitions.h"

#include "SparkFun_Ublox_Arduino_Library.h"
#include "3d_math.h"
#include "CircularBuffer.h"

#include "interval_control.h"
#include "utils/device_status.h"



#define GPS_RATE 100

#define SENSOR_MEASUREMENT_TIMEOUT_US 20000

#define GPS_SERIAL_PORT Serial5



namespace GPS {

    extern CircularBuffer <Vector, 100> positionFifo;
    extern CircularBuffer <Vector, 100> velocityFifo;

    uint8_t getSatellites();

    SFE_UBLOX_GPS* getGPS();

    void deviceThread();

    uint32_t getRate();
    uint32_t getMeasurementRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif