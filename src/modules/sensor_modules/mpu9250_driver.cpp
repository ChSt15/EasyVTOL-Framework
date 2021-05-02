#include "mpu9250_driver.h"



uint32_t MPU9250Driver::_newDataTimestamp = 0;
bool MPU9250Driver::_newDataInterrupt = false;



void MPU9250Driver::_getData() {

    _imu.Read();

    Vector bufVec(-_imu.gyro_x_radps(), _imu.gyro_y_radps(), -_imu.gyro_z_radps());
    if (_lastGyro != bufVec) {
        //Serial.println(String("Gyro: x:") + bufVec.x + ", y:" + bufVec.y + ", z:" + bufVec.z + ", Rate:" + _gyroRate);
        _gyroFifo.push_front(bufVec);
        _gyroTimestampFifo.push_front(_newDataTimestamp);
        _lastGyro = bufVec;
        _gyroCounter++;
    }

    bufVec = Vector(-_imu.accel_x_mps2(), _imu.accel_y_mps2(), -_imu.accel_z_mps2());
    if (_lastAccel != bufVec) {
        _accelFifo.push_front(bufVec);
        _accelTimestampFifo.push_front(_newDataTimestamp);
        _lastAccel = bufVec;
        _accelCounter++;
    }

    if (_imu.MagnetometerFailed()) return; //Do not get mag data if mag failed to start.

    bufVec = Vector(-_imu.mag_x_ut(), _imu.mag_y_ut(), -_imu.mag_z_ut());
    if (_lastMag != bufVec) {
        _magFifo.push_front(bufVec);
        _magTimestampFifo.push_front(_newDataTimestamp);
        _lastMag = bufVec;
        _magCounter++;
    }

}


void MPU9250Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        if (_newDataInterrupt) { //If true then data is ready in the imu FIFO
            _newDataInterrupt = false;

            _getData();
            

        }

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        Serial.println("CALIBRATING IMU");
        //imu.calibrateGyro();
        //imu.calibrateMag();
        for (byte n = 0; n < 6; n++) {
            if (n != 0) Serial.println("SWITCH");
            //_imu.calibrateAccel();
        }

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        _block = true;
        _loopRate = 0;

    }


    uint32_t dTime;
    if (_rateCalcInterval.isTimeToRun(dTime)) {
        float dTime_s = (float)dTime/1000000.0f;
        _loopRate = _loopCounter/dTime_s;
        _gyroRate = _gyroCounter/dTime_s;
        _accelRate = _accelCounter/dTime_s;
        _magRate = _magCounter/dTime_s;
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

    //Serial.println("Test1");
    int startCode = _imu.Begin();
    //Serial.println("Test2");


    if (startCode > 0) {

        if (_imu.MagnetometerFailed()) Serial.println("Magnetometer failed, But gyro and accel are working!!!!");

        _imu.ConfigAccelRange(Mpu9250::AccelRange::ACCEL_RANGE_16G);
        _imu.ConfigGyroRange(Mpu9250::GyroRange::GYRO_RANGE_2000DPS);
        _imu.EnableDrdyInt();

        _imu.ConfigSrd(0);
        _imu.ConfigDlpf(Mpu9250::DlpfBandwidth::DLPF_BANDWIDTH_250HZ_4kHz);


        attachInterrupt(MPU_INT_PIN, _interruptRoutine, RISING);

        _lastMeasurement = micros();
        

        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("NEW! IMU Start Fail. Code: " + String(startCode));
    }

    _startAttempts++;

    if (_startAttempts >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
