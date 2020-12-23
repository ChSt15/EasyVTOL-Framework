#include "imu.h"


namespace IMU {

    namespace {

        const int imuNCS = 3;
        const int imuInt = 2;

        IntervalControl imuInterval; 

        MPU9250FIFO imuTest(SPI, imuNCS);

    }

    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 

}


void IMU::imuThread() {

    if (!imuInterval.isTimeToRun()) return; 


    if (imuStatus == DeviceStatus::DEVICE_RUNNNIG) {



    } else if (imuStatus == DEVICE_NOT_STARTED) {

        imuInterval.setRate(32000);

    } else if (imuStatus == DeviceStatus::DEVICE_FAILURE) {



    }

}


