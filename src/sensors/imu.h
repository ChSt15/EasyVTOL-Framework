#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#include "definitions.h"

#include "mpu9250.h"
#include "3d_math.h"
#include "CircularBuffer.h"
#include "TeensyThreads.h"

#include "interval_control.h"
#include "utils/device_status.h"



#define IMU_RATE_LIMIT 32000

#define SENSOR_MEASUREMENT_TIMEOUT_US 20000

#define GYRO_MEASUREMENT_INTERVAL_US 
#define ACCEL_MEASUREMENT_INTERVAL_US 
#define MAG_MEASUREMENT_INTERVAL_US 


namespace IMU {

    bool gyroAvailable();
    bool getGyro(Vector* gyro, uint32_t* timestamp);
    bool peekGyro(Vector* gyro, uint32_t* timestamp); //TO BE IMPLEMENTED. Peak returns the first elements in fifo but does not remove them.
    bool removeGyro(); //TO BE IMPLEMENTED. Removes the first element in the fifo. Returns true if element was removed. False if no data to be removed

    bool accelAvailable();
    bool getAccel(Vector* accel, uint32_t* timestamp);
    bool peekAccel(Vector* accel, uint32_t* timestamp); //TO BE IMPLEMENTED
    bool removeAccel(); //TO BE IMPLEMENTED

    bool magAvailable();
    bool getMag(Vector* mag, uint32_t* timestamp);
    bool peekMag(Vector* mag, uint32_t* timestamp); //TO BE IMPLEMENTED
    bool removeMag(); //TO BE IMPLEMENTED

    void deviceThread();

    uint32_t getLoopRate();

    uint32_t getGyroRate();
    uint32_t getAccelRate(); 
    uint32_t getMagRate(); 

    DeviceStatus getDeviceStatus();
    
} 





#endif