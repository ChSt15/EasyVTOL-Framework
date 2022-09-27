#include "KraftKontrol/modules/sensor_modules/imu_modules/mpu9250_driver.h"



int64_t MPU9250Driver::_newDataTimestamp = 0;
bool MPU9250Driver::_newDataInterrupt = false;
Task_Abstract* MPU9250Driver::task_ = nullptr;



void MPU9250Driver::_getData() {

    _imu.Read();

    DataTimestamped<Vector<>> gyroVec(Vector<>(_imu.gyro_x_radps(), _imu.gyro_y_radps(), _imu.gyro_z_radps()), _newDataTimestamp);
    if (_lastGyro != gyroVec.data || true) {
        
        DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> buf;
        buf.data.values[0][0] = gyroVec.data.x;
        buf.data.values[1][0] = gyroVec.data.y;
        buf.data.values[2][0] = gyroVec.data.z;
        buf.data.covariance = FML::Matrix<float, 3, 3>::eye(0.002);
        buf.timestamp = gyroVec.timestamp;
        publishGyroData(buf);
        _lastGyro = gyroVec.data;
        _gyroCounter++;

    }

    DataTimestamped<Vector<>> accelVec = DataTimestamped<Vector<>>(Vector<>(_imu.accel_x_mps2(), _imu.accel_y_mps2(), _imu.accel_z_mps2()), _newDataTimestamp);
    //Serial.println(String("Dtime: ") + uint32_t(NOW()-_newDataTimestamp) + ", gyro: " + gyroVec.data.toString() + ", accel: " + accelVec.data.toString());
    //Serial.println(String() + "x:" + gyroVec.data.x + " y:" + gyroVec.data.y + " z: " + gyroVec.data.z);
    if (_lastAccel != accelVec.data || true) {

        DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> buf;
        buf.data.values[0][0] = accelVec.data.x;
        buf.data.values[1][0] = accelVec.data.y;
        buf.data.values[2][0] = accelVec.data.z;
        buf.data.covariance = FML::Matrix<float, 3, 3>::eye(0.07);
        buf.timestamp = accelVec.timestamp;

        publishAccelData(buf);
        _lastAccel = accelVec.data;
        _accelCounter++;
        
    }

}


void MPU9250Driver::thread() {


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
        stopTaskThreading();

    }


    int64_t dTime;
    if (_rateCalcInterval.isTimeToRun(dTime)) {
        float dTime_s = (float)dTime/SECONDS;
        _gyroRate = _gyroCounter/dTime_s;
        _accelRate = _accelCounter/dTime_s;
        _magRate = _magCounter/dTime_s;
        _gyroCounter = 0;
        _accelCounter = 0;
        _magCounter = 0;
    }

    this->stopTaskThreading();

}



void MPU9250Driver::_interruptRoutine() {
    _newDataInterrupt = true;
    _newDataTimestamp =  NOW();
    task_->startTaskThreading();
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
        _imu.ConfigDlpf(Mpu9250::DlpfBandwidth::DLPF_BANDWIDTH_184HZ);

        _lastMeasurement = NOW();

        pinInterrupt_.setEnable(true);
        
        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("IMU Start Fail. Code: " + String(startCode));
    }

    _startAttempts++;

    if (_startAttempts >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
