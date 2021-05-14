#include "ublox_serial_gnss.h"



void UbloxSerialGNSS::_getData() {

    uint32_t time = micros();

    numSats_ = gnss_.getSIV();

    WorldPosition position;
    position.latitude = (double)gnss_.getLatitude()*10e7;
    position.longitude = (double)gnss_.getLongitude()*10e7;
    position.longitude = (float)gnss_.getAltitudeMSL()/1000;

    positionFifo_.push_front(position);
    positionTimestampFifo_.push_front(time);


    Vector velocity;
    velocity.x = (float)gnss_.getNedNorthVel()/1000;
    velocity.y = -(float)gnss_.getNedEastVel()/1000;
    velocity.z = -(float)gnss_.getNedDownVel()/1000;

    velocityFifo_.push_front(velocity);
    velocityTimestampFifo_.push_front(time);

    positionDeviation_ = gnss_.getHorizontalAccuracy();
    altitudeDeviation_ = gnss_.getVerticalAccuracy();

    positionCounter_++;
    velocityCounter_++;

}


void UbloxSerialGNSS::thread() {


    if (usbPassthrough_) {
        
        while (serialPort_->available()) Serial.write(serialPort_->read());
        while (Serial.available()) serialPort_->write(Serial.read());
        
        return;
    }


    loopCounter_++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        //gnss_.getPVT();

        if (gnss_.getPVT()) { //If high then data is ready in the gps FIFO

            _getData();

        }

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Starting) {
        


    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        loopRate_ = 0;

        stopTaskThreading();

    }


    uint32_t dTime = 0;
    if (rateCalcInterval_.isTimeToRun(dTime)) {
        loopRate_ = loopCounter_;
        velocityRate_ = velocityCounter_;
        positionRate_ = positionCounter_;
        loopCounter_ = positionCounter_ = velocityCounter_ = 0;
    }

}



void UbloxSerialGNSS::init() {

    if (usbPassthrough_) {
        serialPort_->begin(115200);
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;
        return;
    }

    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted) serialPort_->begin(115200);
    else serialPort_->begin(9600*max(serialBaudMulti_,1));

    if (gnss_.begin(*serialPort_)) {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        gnss_.setSerialRate(115200);
        serialPort_->begin(115200);

        gnss_.setUART1Output(COM_TYPE_UBX);
        gnss_.setNavigationFrequency(10);
        //gnss_.setAutoPVT(true);
        gnss_.assumeAutoPVT(true, true);
        gnss_.setDynamicModel(DYN_MODEL_AIRBORNE4g);

        gnss_.saveConfiguration();

    } else {

        if (moduleStatus_ != eModuleStatus_t::eModuleStatus_NotStarted) {
            serialBaudMulti_ += 2;
        }

        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt;

        if (serialBaudMulti_ >= 14) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

    }

}

