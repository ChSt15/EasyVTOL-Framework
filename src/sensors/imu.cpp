#include "imu.h"


namespace IMU {

    namespace {

        const int imuNCS = 3;
        const int imuInt = 2;

        IntervalControl imuInterval(32000); 
        IntervalControl rateCalcInterval(1); 


        MPU9250FIFO imuTest(SPI, imuNCS);

    }

    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 

    uint32_t rate = 0;
    uint32_t loopCounter = 0;


}


void IMU::imuThread() {

    if (!imuInterval.isTimeToRun()) return; 

    loopCounter++;


    if (imuStatus == DeviceStatus::DEVICE_RUNNING) {



    } else if (imuStatus == DEVICE_NOT_STARTED) {

        imuStatus = DeviceStatus::DEVICE_RUNNING;

    } else if (imuStatus == DeviceStatus::DEVICE_FAILURE) {



    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint32_t IMU::getRate() {
    return rate;
}


