#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#include "MPU9250-master/src/MPU9250.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



namespace IMU {

    void imuThread();
    
} 





#endif