#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/hmc5883_driver.h"



bool QMC5883Driver::getData() {

    uint8_t buffer[6];

    if (!bus_->readBytes(QMC5883Registers::QMC5883L_X_LSB, buffer, 6)) return false;

    int64_t time = NOW();

    int16_t x,y,z;

    x = static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1])<<8);
    y = static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3])<<8);
    z = static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5])<<8);


    /*if (calibrate_) {

        if (calibrationStatus_ != eMagCalibStatus_t::eMagCalibStatus_Calibrating) {
            calibrationStatus_ = eMagCalibStatus_t::eMagCalibStatus_Calibrating;

            magMin_ = 900000;
            magMax_ = -900000;

        }


        if (mag.x > magMax_.x) {
            magMax_.x = mag.x;
        }
        if (mag.x < magMin_.x) {
            magMin_.x = mag.x;
        }
        if (mag.y > magMax_.y) {
            magMax_.y = mag.y;
        }
        if (mag.y < magMin_.y) {
            magMin_.y = mag.y;
        }
        if (mag.z > magMax_.z) {
            magMax_.z = mag.z;
        }
        if (mag.z < magMin_.z) {
            magMin_.z = mag.z;
        }

        Serial.println(String("Min: ") + magMin_.toString() + ", Max: " + magMax_.toString());

        //timeout for calibration
        if (NOW() - calibrationStart_ >= 60*SECONDS) stopCalibration();


    } else*/ 


    DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> buf;
    buf.data.values[0][0] = x*8.0f/32767.0f;
    buf.data.values[1][0] = y*8.0f/32767.0f;
    buf.data.values[2][0] = z*8.0f/32767.0f;
    buf.data.covariance = FML::Matrix<float, 3, 3>::eye(0.006);
    buf.timestamp = time;

    publishMagData(buf);

    //Serial.println(String("Min: ") + magMin_.toString() + ", Max: " + magMax_.toString());

    //Serial.println(String("Mag: x: ") + buf.data.values[0][0] + ", y: " + buf.data.values[1][0] + ", z: " + buf.data.values[2][0]);

    //bus_->readBytes(QMC5883Registers::QMC5883L_TEMP_LSB, buffer, 2);

    //memcpy(&x, buffer, 2);

    //x = static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1])<<8);

    //float temp = static_cast<float>(x)/10.0f;

    //Serial.println(String("temp: ") + temp);

    return true;

}


bool QMC5883Driver::dataAvailable() {

    uint8_t byte = 0;

    if (!bus_->readByte(QMC5883Registers::QMC5883L_STATUS, byte)) return false;

    return (byte&B00000001) == B00000001;

}


void QMC5883Driver::thread() {


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        if (dataAvailable()) {

            lastMeasurement_ = NOW();

            if (!getData()) {

                //moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
                suspendUntil(END_OF_TIME);

            }

        } else if (NOW() - lastMeasurement_ > 200*MILLISECONDS) {

            //moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
            //stopTaskThreading();

        }

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        suspendUntil(END_OF_TIME);

    }

}



void QMC5883Driver::init() {

    uint32_t failed = 0;

    //uint8_t byte;
    //if (!readByte(address_, QMC5883Registers::QMC5883L_CHIP_ID, &byte)) failed = true;

    if (failed == 0 && !bus_->writeByte(QMC5883Registers::QMC5883L_CONFIG2, 0b10000000)) failed = 1;

    delay(10);

    if (failed == 0 && !bus_->writeByte(QMC5883Registers::QMC5883L_RESET, 0x01)) failed = 4;
    if (failed == 0 && !bus_->writeByte(QMC5883Registers::QMC5883L_CONFIG, 0b00011101)) failed = 2;
    if (failed == 0 && !bus_->writeByte(QMC5883Registers::QMC5883L_CONFIG2, 0b00000000)) failed = 3;


    if (failed == 0) {

        Serial.println(String("mag start success!"));

        lastMeasurement_ = NOW();

        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    } else {
        Serial.println(String("mag start FAILED! Code: ") + failed);
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
    }

    _startAttempts++;

    if (_startAttempts >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
