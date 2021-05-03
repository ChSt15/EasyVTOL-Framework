#include "gps.h"
/*

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
    uint32_t sensorRate = 0;
    uint32_t sensorCounter = 0;

    uint32_t lastMeasurement = 0;
    uint32_t lastGPSTime = 0;

}


void GPS::deviceThread() {

    if (!gpsInterval.isTimeToRun()) return; 

    loopCounter++;


    if (gpsStatus == DeviceStatus::DEVICE_RUNNING) {

        gps.getPVT();

        if (gps.timeOfWeek != lastGPSTime) { //If high then data is ready in the gps FIFO
            lastGPSTime = gps.timeOfWeek;

            numSatellites = gps.SIV;

            sensorCounter++;

        }

    } else if (gpsStatus == DeviceStatus::DEVICE_NOT_STARTED || gpsStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {
        
        if (gpsStatus == DeviceStatus::DEVICE_NOT_STARTED) GPS_SERIAL_PORT.begin(115200);
        else GPS_SERIAL_PORT.begin(9600*max(serialBaudMulti,1));

        if (gps.begin(GPS_SERIAL_PORT)) {

            gpsInterval.setRate(GPS_RATE);
            gpsStatus = DeviceStatus::DEVICE_RUNNING;

            gps.setSerialRate(115200);
            GPS_SERIAL_PORT.begin(115200);

            gps.setNavigationFrequency(10);
            gps.setAutoPVT(true);
            gps.setDynamicModel(DYN_MODEL_AIRBORNE4g);

            gps.saveConfiguration();

            lastMeasurement = micros();

        } else {

            if (gpsStatus != DeviceStatus::DEVICE_NOT_STARTED) {
                serialBaudMulti += 2;
            }
            gpsStatus = DeviceStatus::DEVICE_RESTARTATTEMPT;

            if (serialBaudMulti >= 14) gpsStatus = DeviceStatus::DEVICE_FAILURE;

        }

    } else if (gpsStatus == DeviceStatus::DEVICE_CALIBRATING) {
        


    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        gpsStatus = DeviceStatus::DEVICE_FAILURE;
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        sensorRate = sensorCounter;
        loopCounter = 0;
        sensorCounter = 0;
    }

}


uint8_t GPS::getSatellites() {return numSatellites;}


SFE_UBLOX_GPS* GPS::getGPS() {return &gps;}


uint32_t GPS::getRate() {
    return rate;
}


uint32_t GPS::getMeasurementRate() {
    return sensorRate;
}


DeviceStatus GPS::getDeviceStatus() {return gpsStatus;}


*/