#include "gps.h"


namespace GPS {

    CircularBuffer <Vector, 100> positionFifo;
    CircularBuffer <Vector, 100> velocityFifo;

    uint8_t numSatellites = 0;

    IntervalControl gpsInterval(0.2); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    SFE_UBLOX_GPS gps;
    uint32_t serialBaudMulti = 0;

    DeviceStatus gpsStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    uint32_t lastMeasurement = 0;

}


void GPS::deviceThread() {

    if (!gpsInterval.isTimeToRun()) return; 

    loopCounter++;


    if (gpsStatus == DeviceStatus::DEVICE_RUNNING) {

        uint32_t t0 = micros();

        if (gps.getPVT()) { //If high then data is ready in the gps FIFO

            numSatellites = gps.getSIV();

        }

    } else if (gpsStatus == DeviceStatus::DEVICE_NOT_STARTED || gpsStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        Serial2.begin(9600*max(serialBaudMulti,1));

        Serial.println("Attempting Start at: " + String(9600*max(serialBaudMulti,1)));

        if (gps.begin(Serial2)) {

            gpsInterval.setRate(GPS_RATE);
            gpsStatus = DeviceStatus::DEVICE_RUNNING;

            gps.setSerialRate(115200);
            Serial2.begin(115200);

            gps.setNavigationFrequency(10);
            gps.setAutoPVT(true);
            gps.saveConfiguration();

            lastMeasurement = micros();

        } else {

            gpsStatus = DeviceStatus::DEVICE_RESTARTATTEMPT;
            serialBaudMulti += 2;

            if (serialBaudMulti >= 14) gpsStatus = DeviceStatus::DEVICE_FAILURE;

        }

    } else if (gpsStatus == DeviceStatus::DEVICE_CALIBRATING) {
        


    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        gpsStatus = DeviceStatus::DEVICE_FAILURE;
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint8_t GPS::getSatellites() {return numSatellites;}


SFE_UBLOX_GPS* GPS::getGPS() {return &gps;}


uint32_t GPS::getRate() {
    return rate;
}


DeviceStatus GPS::getDeviceStatus() {return gpsStatus;}


