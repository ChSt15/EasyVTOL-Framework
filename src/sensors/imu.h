#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#include "MPU9250.h"
#include "vector_math.h"
#include "CircularBuffer.h"
#include "TeensyThreads.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define IMU_RATE 8000



namespace IMU {

    extern volatile CircularBuffer <Vector, 100> gyroFifo;
    extern volatile CircularBuffer <Vector, 100> accelFifo;
    extern volatile CircularBuffer <Vector, 100> magFifo;

    extern Vector lastGyro;
    extern Vector lastAccel;
    extern Vector lastMag;

    extern bool newData;

    void deviceThread();

    uint32_t getRate();

    uint32_t getMeasurementRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif