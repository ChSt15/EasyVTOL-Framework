#include "mpu9250_driver.h"



uint32_t MPU9250Driver::_newDataTimestamp = 0;
bool MPU9250Driver::_newDataInterrupt = false;

MPU9250Driver IMU; //For outside use


void MPU9250Driver::_getData() {

    _imu.Read();

    Vector bufVec(-_imu.gyro_x_radps(), _imu.gyro_y_radps(), -_imu.gyro_z_radps());
    if (_lastGyro != bufVec) {
        //Serial.println(String("Gyro: x:") + bufVec.x + ", y:" + bufVec.y + ", z:" + bufVec.z + ", Rate:" + _gyroRate);
        _gyroFifo.unshift(bufVec);
        _gyroTimestampFifo.unshift(_newDataTimestamp);
        _lastGyro = bufVec;
        _gyroCounter++;
    }

    bufVec = Vector(_imu.accel_x_mps2(), -_imu.accel_y_mps2(), _imu.accel_z_mps2());
    if (_lastAccel != bufVec) {
        _accelFifo.unshift(bufVec);
        _accelTimestampFifo.unshift(_newDataTimestamp);
        _lastAccel = bufVec;
        _accelCounter++;
    }

    bufVec = Vector(-_imu.mag_x_ut(), _imu.mag_y_ut(), -_imu.mag_z_ut());
    if (_lastMag != bufVec) {
        _magFifo.unshift(bufVec);
        _magTimestampFifo.unshift(_newDataTimestamp);
        _lastMag = bufVec;
        _magCounter++;
    }

}


void MPU9250Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (_moduleStatus == MODULE_STATUS::MODULE_RUNNING) {

        if (_newDataInterrupt) { //If true then data is ready in the imu FIFO
            _newDataInterrupt = false;

            _getData();
            

        }

    } else if (_moduleStatus == MODULE_STATUS::MODULE_NOT_STARTED || _moduleStatus == MODULE_STATUS::MODULE_RESTARTATTEMPT) {
        
        //Serial.println("Test1");
        int startCode = _imu.Begin();
        //Serial.println("Test2");


        if (startCode > 0) {

            /*imu.setMagCalX(18.26, 1.0f);
            imu.setMagCalY(39.14, 1.0f);
            imu.setMagCalZ(-40.69, 1.0f);

            imu.setAccelCalX(0.20268, 1.0f);
            imu.setAccelCalY(-0.14238, 1.001f);
            imu.setAccelCalZ(0.6755, 0.9869f);*/

            _imu.ConfigAccelRange(Mpu9250::AccelRange::ACCEL_RANGE_16G);
            _imu.ConfigGyroRange(Mpu9250::GyroRange::GYRO_RANGE_2000DPS);
            _imu.EnableDrdyInt();

            _imu.ConfigSrd(0);
            _imu.ConfigDlpf(Mpu9250::DlpfBandwidth::DLPF_BANDWIDTH_184HZ);


            attachInterrupt(MPU_INT_PIN, _interruptRoutine, HIGH);

            _lastMeasurement = micros();
            

            //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
            _moduleStatus = MODULE_STATUS::MODULE_RUNNING;

        } else {
            _moduleStatus = MODULE_STATUS::MODULE_RESTARTATTEMPT; 
            Serial.println("NEW! IMU Start Fail. Code: " + String(startCode));
            delay(1000);
        }

        _startAttempts++;

        if (_startAttempts >= 5 && _moduleStatus == MODULE_STATUS::MODULE_RESTARTATTEMPT) _moduleStatus = MODULE_STATUS::MODULE_FAILURE;

    } else if (false/*_moduleStatus == MODULE_STATUS::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        Serial.println("CALIBRATING IMU");
        //imu.calibrateGyro();
        //imu.calibrateMag();
        for (byte n = 0; n < 6; n++) {
            if (n != 0) Serial.println("SWITCH");
            //_imu.calibrateAccel();
        }

        _moduleStatus = MODULE_STATUS::MODULE_RUNNING; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        _moduleStatus = MODULE_STATUS::MODULE_FAILURE;
        _block = true;
        _loopRate = 0;

    }



    if (_rateCalcInterval.isTimeToRun()) {
        _loopRate = _loopCounter;
        _gyroRate = _gyroCounter;
        _accelRate = _accelCounter;
        _magRate = _magCounter;
        _gyroCounter = 0;
        _accelCounter = 0;
        _magCounter = 0;
        _loopCounter = 0;
    }

}



void MPU9250Driver::_interruptRoutine() {
    _newDataInterrupt = true;
    _newDataTimestamp = micros();
}



void MPU9250Driver::init() {



}
