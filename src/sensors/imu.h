#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#include "MPU9250.h"
#include "vector_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



namespace IMU {

    extern CircularBuffer <Vector, 100> gyroFifo;
    extern CircularBuffer <Vector, 100> accelFifo;
    extern CircularBuffer <Vector, 100> magFifo;

    uint16_t getDataCount();

    void getData(Vector *accel, Vector *gyro, Vector *mag, uint16_t size);

    void imuThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif