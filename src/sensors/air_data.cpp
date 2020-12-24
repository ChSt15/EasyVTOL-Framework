#include "air_data.h"


namespace AirData {

    const int bmeNCS = 6;

    CircularBuffer <float, 50> pressureFifo;
    CircularBuffer <float, 50> humidityFifo;
    CircularBuffer <float, 50> temperatureFifo;
    CircularBuffer <uint32_t, 50> timestampFifo;

    IntervalControl bmeInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    BME280 bme;

    DeviceStatus bmeStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    uint32_t lastMeasurement = 0;

}


void AirData::deviceThread() {

    if (!bmeInterval.isTimeToRun()) return; 

    loopCounter++;


    if (bmeStatus == DeviceStatus::DEVICE_RUNNING) {

        if (!bme.isMeasuring()) {

            pressureFifo.unshift(bme.readFloatPressure());
            humidityFifo.unshift(bme.readFloatHumidity());
            temperatureFifo.unshift(bme.readTempC());

            timestampFifo.unshift(micros());

            lastMeasurement = micros();

        } else if (micros() - lastMeasurement >= SENSOR_MEASUREMENT_TIMEOUT_US) bmeStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (bmeStatus == DeviceStatus::DEVICE_NOT_STARTED || bmeStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        if (bme.beginSPI(bmeNCS)) {

            bmeInterval.setRate(BME_RATE);

            bme.setHumidityOverSample(2);
            bme.setPressureOverSample(2);
            bme.setTempOverSample(2);
            bme.setFilter(4);
            bme.setMode(MODE_NORMAL);
            bme.setStandbyTime(7);

            lastMeasurement = micros();

            bmeStatus = DeviceStatus::DEVICE_RUNNING; //Skip calibration

        } else bmeStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 

        startAttempts++;

        if (startAttempts >= 5 && bmeStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) bmeStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (bmeStatus == DeviceStatus::DEVICE_CALIBRATING) {

        

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        bmeStatus = DeviceStatus::DEVICE_FAILURE;
        bmeInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint32_t AirData::getRate() {
    return rate;
}


DeviceStatus AirData::getDeviceStatus() {return bmeStatus;}


