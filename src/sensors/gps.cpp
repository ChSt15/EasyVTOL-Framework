#include "gps.h"


namespace GPS {

    CircularBuffer <Vector, 100> positionFifo;
    CircularBuffer <Vector, 100> velocityFifo;

    uint8_t numSatellites = 0;

    IntervalControl gpsInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    UBLOX gps(Serial2, 115200);

    DeviceStatus gpsStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    uint32_t lastMeasurement = 0;

    void readData();

}


void GPS::readData() {

    Serial.println("GPS DATA READ!");

    numSatellites = gps.getNumSatellites();

    Serial.println("Sats: " + String(gps.getSec()));

    lastMeasurement = micros();

}


void GPS::deviceThread() {

    if (!gpsInterval.isTimeToRun()) return; 

    loopCounter++;


    if (gpsStatus == DeviceStatus::DEVICE_RUNNING) {

        if (gps.readSensor()) { //If high then data is ready in the gps FIFO

            readData();

        }

    } else if (gpsStatus == DeviceStatus::DEVICE_NOT_STARTED || gpsStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        gps.begin();

        gpsInterval.setRate(GPS_RATE);

        lastMeasurement = micros();

        gpsStatus = DeviceStatus::DEVICE_RUNNING;

    } else if (gpsStatus == DeviceStatus::DEVICE_CALIBRATING) {
        


    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        gpsStatus = DeviceStatus::DEVICE_FAILURE;
        rate = 0;

        if (gps.readSensor()) {
            readData();
            gpsStatus = DeviceStatus::DEVICE_RUNNING;
        }

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint8_t GPS::getSatellites() {return numSatellites;}


UBLOX* GPS::getGPS() {return &gps;}


uint32_t GPS::getRate() {
    return rate;
}


DeviceStatus GPS::getDeviceStatus() {return gpsStatus;}


