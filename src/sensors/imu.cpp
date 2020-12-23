#include "imu.h"


namespace IMU {

    namespace {

        IntervalControl imuInterval; 

    }


    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 

}


void IMU::imuThread() {

    if (!imuInterval.isTimeToRun()) return; 


    if (imuStatus == DeviceStatus::DEVICE_RUNNNIG) {



    } else if (imuStatus == DEVICE_NOT_STARTED) {

        imuInterval.setRate(1000);

    } else if (imuStatus == DeviceStatus::DEVICE_FAILURE) {



    }

}


