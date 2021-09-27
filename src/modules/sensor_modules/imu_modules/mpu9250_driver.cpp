#include "KraftKontrol/modules/sensor_modules/imu_modules/mpu9250_driver.h"



int64_t MPU9250Driver::_newDataTimestamp = 0;
bool MPU9250Driver::_newDataInterrupt = false;



bool MPU9250Driver::getEEPROMData() {

    if (eeprom_ == nullptr) return false;

    CommandMessageAccelCalValues accelValues;

    //Serial.println("Trying to retrieve accel calib data.");

    if (!eeprom_->getMessage(accelValues)) {

        //Serial.println("Failed to retreive!");

        return false;

    }

    accelMin_ = accelValues.getMinValue();
    accelMax_ = accelValues.getMaxValue();

    //Serial.println(String("Got values. Max: ") + accelMax_.toString() + ", Min: " + accelMin_.toString());

    return true;

}


/*bool MPU9250Driver::setEEPROMData() {

    if (eeprom_ == nullptr) return false;

    CommandMessageAccelCalValues accelValues(accelMax_, accelMin_);

    if (!eeprom_->setMessage(accelValues)) {

        //Failed see if we can create a new message. If not then return false.
        if (!eeprom_->newMessage(accelValues)) {
            return false;
        }

    }

    return true;

}*/


void MPU9250Driver::_getData() {

    _imu.Read();

    SensorTimestamp<Vector<>> bufVec(Vector<>(-_imu.gyro_x_radps(), _imu.gyro_y_radps(), -_imu.gyro_z_radps()), _newDataTimestamp);
    if (_lastGyro != bufVec.sensorData || true) {
        //Serial.println(String("Gyro: x:") + bufVec.x + ", y:" + bufVec.y + ", z:" + bufVec.z + ", Rate:" + _gyroRate);
        gyroTopic_.publish(bufVec);
        _lastGyro = bufVec.sensorData;
        _gyroCounter++;
    }

    bufVec = SensorTimestamp<Vector<>>(Vector<>(-_imu.accel_x_mps2(), _imu.accel_y_mps2(), -_imu.accel_z_mps2()), _newDataTimestamp);
    if (_lastAccel != bufVec.sensorData || true) {

        /*if (calibrate_) {

            Serial.println(bufVec.sensorData.toString());

            switch (accelCalibState_)
            {
            case 0:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                    accelMax_.x = calibBuf_.getMedian();
                }
                break;

            case 1:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                }
                break;

            case 2:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                }
                break;

            case 3:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                }
                break;

            case 4:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                }
                break;

            case 5:
                calibBuf_.placeFront(bufVec.sensorData.x, true);
                if (NOW() - accelCalibStateTime_ >= 10*SECONDS) {
                    accelCalibState_ = 1;
                    accelCalibStateTime_ = NOW();
                }
                break;
            
            default:
                break;
            }

        }*/

        bufVec.sensorData = ((bufVec.sensorData - accelMin_)/(accelMax_ - accelMin_)*2-1)*9.81;

        accelTopic_.publish(bufVec);
        _lastAccel = bufVec.sensorData;
        _accelCounter++;
    }

    if (_imu.MagnetometerFailed()) return; //Do not get mag data if mag failed to start.

    bufVec = SensorTimestamp<Vector<>>(Vector<>(-_imu.mag_x_ut(), _imu.mag_y_ut(), -_imu.mag_z_ut()), _newDataTimestamp);
    if (_lastMag != bufVec.sensorData || true) {
        //topic.publish(bufVec);
        _lastMag = bufVec.sensorData;
        _magCounter++;
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

}



void MPU9250Driver::_interruptRoutine() {
    _newDataInterrupt = true;
    _newDataTimestamp =  NOW();
}



void MPU9250Driver::init() {

    //Serial.println("Test1");
    int startCode = _imu.Begin();
    //Serial.println("Test2");


    if (startCode > 0) {

        if (_imu.MagnetometerFailed()) Serial.println("Magnetometer failed, But gyro and accel are working!!!!");

        _imu.ConfigAccelRange(Mpu9250::AccelRange::ACCEL_RANGE_8G);
        _imu.ConfigGyroRange(Mpu9250::GyroRange::GYRO_RANGE_2000DPS);
        _imu.EnableDrdyInt();

        _imu.ConfigSrd(0);
        _imu.ConfigDlpf(Mpu9250::DlpfBandwidth::DLPF_BANDWIDTH_250HZ_4kHz);

        _lastMeasurement = NOW();

        //Check for calibration in EEPROM.
        if (getEEPROMData()) {
            //calibrationStatus_ = eMagCalibStatus_t::eMagCalibStatus_Calibrated;
        }// else startCalibration();

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
