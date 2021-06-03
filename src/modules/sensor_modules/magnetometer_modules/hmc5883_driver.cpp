#include "hmc5883_driver.h"



void QMC5883Driver::getData() {

    uint8_t buffer[6];

    if (!readBytes(address_, QMC5883Registers::QMC5883L_X_LSB, buffer, 6)) return;

    int16_t x,y,z;

    x = static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1])<<8);
    y = static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3])<<8);
    z = static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5])<<8);

    Vector<> mag = Vector<>(x,y,z)*8.0f/32767.0f;

    magFifo_.placeFront(mag, true);
    magTimestampFifo_.placeFront(micros(), true);

    //readBytes(address_, QMC5883Registers::QMC5883L_TEMP_LSB, buffer, 2);

    //memcpy(&x, buffer, 2);

    //x = static_cast<int16_t>(buffer[1]) | (static_cast<int16_t>(buffer[0])<<8);

    //float temp = static_cast<float>(x)/10.0f;

    //Serial.println(String("temp: ") + temp);

    magCounter_++;

}


bool QMC5883Driver::dataAvailable() {

    uint8_t byte = 0;

    if (!readByte(address_, QMC5883Registers::QMC5883L_STATUS, &byte)) return false;

    return (byte&0x01) == 0x01;

}


void QMC5883Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        if (dataAvailable()) getData();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

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
        _magRate = magCounter_/dTime_s;
        magCounter_ = 0;
        _loopCounter = 0;
    }

}



void QMC5883Driver::init() {

    bool failed = false;

    uint8_t byte;

    //if (!readByte(address_, QMC5883Registers::QMC5883L_CHIP_ID, &byte)) failed = true;

    if (!writeByte(address_, QMC5883Registers::QMC5883L_CONFIG2, 0b10000000)) failed = true;

    delay(10);

    if (!writeByte(address_, QMC5883Registers::QMC5883L_CONFIG, 0b00101001)) failed = true;
    if (!writeByte(address_, QMC5883Registers::QMC5883L_CONFIG2, 0b00000000)) failed = true;
    if (!writeByte(address_, QMC5883Registers::QMC5883L_RESET, 0x01)) failed = true;


    if (!failed) {

        Serial.println(String("mag start success!"));

        _lastMeasurement = micros();

        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    } else {
        Serial.println(String("mag start FAILED!"));
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
    }

    _startAttempts++;

    if (_startAttempts >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
