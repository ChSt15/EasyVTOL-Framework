#include "KraftKontrol/modules/sensor_modules/gnss_modules/ublox_serial_gnss.h"



void UbloxSerialGNSS::_getData() {

    int64_t time = NOW();

    numSats_ = gnss_.getSIV();

    positionDeviation_ = (float)gnss_.getHorizontalAccEst()/1000.0f;
    altitudeDeviation_ = (float)gnss_.getVerticalAccEst()/1000.0f;

    //gnss_.getSurveyInMeanAccuracy();

    //gnss_.getPositionAccuracy();

    //if ()

    WorldPosition position;
    position.latitude = (double)gnss_.getLatitude()/10e6*DEGREES;
    position.longitude = (double)gnss_.getLongitude()/10e6*DEGREES;
    position.height = (float)gnss_.getAltitudeMSL()/1000;

    Vector<> velocity;
    velocity.x = (float)gnss_.getNedNorthVel()/1000;
    velocity.y = -(float)gnss_.getNedEastVel()/1000;
    velocity.z = -(float)gnss_.getNedDownVel()/1000;

    //positionDeviation_ = gnss_.getHorizontalAccuracy(0);
    //altitudeDeviation_ = gnss_.getVerticalAccuracy(0);

    positionCounter_++;
    velocityCounter_++;

    if (numSats_ >= minNumSats_ && positionDeviation_ < 100 && altitudeDeviation_ < 100) {
        lockValid_ = true;
    } else {
        lockValid_ = false;
    }

    GNSSData data;
    data.lockValid = lockValid_;
    data.altitudeError = altitudeDeviation_;
    data.positionError = positionDeviation_;
    data.positionValueTimestamp.sensorData = position;
    data.velocityValueTimestamp.sensorData = velocity;
    data.positionValueTimestamp.sensorTimestamp = data.velocityValueTimestamp.sensorTimestamp = time;

    gnssTopic_.publish(data);

}


void UbloxSerialGNSS::thread() {


    if (usbPassthrough_) {
        
        while (serialPort_->available()) Serial.write(serialPort_->read());
        while (Serial.available()) serialPort_->write(Serial.read());
        
        return;
    }


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        //gnss_.getPVT();

        if (gnss_.getPVT()) { //If high then data is ready in the gps FIFO

            _getData();
            lastMeasurement_ = micros();

        } else if (micros() - lastMeasurement_ >= 500000) {

            //gnss_.factoryDefault(100);
            //init();
            gnss_.flushPVT();
            lastMeasurement_ = micros();

        }

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Starting) {
        


    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

        stopTaskThreading();

    }


    uint32_t dTime = 0;
    if (rateCalcInterval_.isTimeToRun(dTime)) {
        velocityRate_ = velocityCounter_;
        positionRate_ = positionCounter_;
        positionCounter_ = velocityCounter_ = 0;
    }

}



void UbloxSerialGNSS::setupSerial(uint32_t baudRate) {

    #ifdef ESP32 

    serialPort_->begin(baudRate, 134217756U, rxPin_, txPin_);

    #elif

    serialPort_->begin(baudRate);

    #endif

}



void UbloxSerialGNSS::init() {

    if (usbPassthrough_) {
        serialPort_->begin(115200);
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;
        return;
    }

    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted) setupSerial(115200);
    else setupSerial(9600*max(serialBaudMulti_,uint32_t(1)));

    if (gnss_.begin(*serialPort_)) {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        gnss_.setSerialRate(115200);
        setupSerial(115200);

        gnss_.setUART1Output(COM_TYPE_UBX);
        gnss_.setNavigationFrequency(10);
        gnss_.setAutoPVT(true);
        gnss_.setDynamicModel(DYN_MODEL_AIRBORNE2g);
        gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GPS);
        gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GALILEO);
        gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GLONASS);
        gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_IMES);
        gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_QZSS);
        gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_SBAS);
        gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_BEIDOU);

        gnss_.saveConfiguration();

        //gnss_.setUART1Output(COM_TYPE_UBX & COM_TYPE_NMEA & COM_TYPE_RTCM3);
        
        //gnss_.setMeasurementRate(10);
        //gnss_.setNavigationRate(10);
        //gnss_.setAutoPVT(true);
        //gnss_.assumeAutoPVT(true, true);
        //gnss_.setDynamicModel(DYN_MODEL_AIRBORNE4g);

        //gnss_.saveConfiguration();

    } else {

        if (moduleStatus_ != eModuleStatus_t::eModuleStatus_NotStarted) {
            serialBaudMulti_ += 2;
        }

        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt;

        if (serialBaudMulti_ >= 14) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

    }

}

