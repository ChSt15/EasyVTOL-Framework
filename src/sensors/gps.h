#ifndef GPS_H
#define GPS_H

#include "Arduino.h"

#include "UBLOX.h"
#include "vector_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define GPS_RATE 100



namespace GPS {

    extern CircularBuffer <Vector, 100> positionFifo;
    extern CircularBuffer <Vector, 100> velocityFifo;

    uint8_t getSatellites();

    UBLOX* getGPS();

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif