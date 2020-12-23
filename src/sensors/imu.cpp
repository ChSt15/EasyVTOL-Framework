#include "imu.h"


namespace IMU {

    namespace {

        const int imuNCS = 3;
        const int imuInt = 2;

        IntervalControl imuInterval; 

        MPU9250FIFO imuTest(SPI, imuNCS);

    }

    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 

    volatile uint32_t counter = 0;

}


void IMU::imuThread() {

    if (!imuInterval.isTimeToRun()) return; 

    counter++;


    if (imuStatus == DeviceStatus::DEVICE_RUNNING) {



    } else if (imuStatus == DEVICE_NOT_STARTED) {

        imuInterval.setRate(32000);
        imuStatus = DeviceStatus::DEVICE_RUNNING;

    } else if (imuStatus == DeviceStatus::DEVICE_FAILURE) {



    }

}


