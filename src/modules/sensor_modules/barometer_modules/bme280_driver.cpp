#include "KraftKontrol/modules/sensor_modules/barometer_modules/bme280_driver.h"



void BME280Driver::getData() {

    BME280_SensorMeasurements measurements;

    uint32_t _newDataTimestamp = micros();
    _bme.readAllMeasurements(&measurements);

    float bufMeasurement = measurements.pressure;
    if (bufMeasurement > 100) {
        _pressureFifo.placeFront(bufMeasurement, true);
        _pressureTimestampFifo.placeFront(_newDataTimestamp, true);
        _lastPressure = bufMeasurement;
        _pressureCounter++;
    }

    bufMeasurement = measurements.temperature;
    if (true) {
        _temperatureFifo.placeFront(bufMeasurement, true);
        _temperatureTimestampFifo.placeFront(_newDataTimestamp, true);
        _lastTemperature = bufMeasurement;
        _temperatureCounter++;
    }

    bufMeasurement = measurements.humidity;
    if (true) {
        _humidityFifo.placeFront(bufMeasurement, true);
        _humidityTimestampFifo.placeFront(_newDataTimestamp, true);
        _lastHumidity = bufMeasurement;
        _humidityCounter++;
    }

}


void BME280Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        /*bool measuring = _bme.isMeasuring();

        if (!sensorMeasuring_ && measuring) {
            sensorMeasuring_ = true;
        } else if (sensorMeasuring_ && !measuring) {
            sensorMeasuring_ = false;
            getData();
        }*/

        getData();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        _block = true;
        _loopRate = 0;

    }



    if (_rateCalcInterval.isTimeToRun()) {
        _loopRate = _loopCounter;
        _pressureRate = _pressureCounter;
        _temperatureRate = _temperatureCounter;
        _humidityRate = _humidityCounter;
        _pressureCounter = 0;
        _temperatureCounter = 0;
        _humidityCounter = 0;
        _loopCounter = 0;
    }

}



void BME280Driver::init() {

    int startCode;

    if (_startAttempts == 0) {
        _bme.reset();
    }

    if (useSPI_) startCode = _bme.beginSPI(chipSelectPin_);
    else {
        _bme.setI2CAddress(i2cAddress_);
        startCode = _bme.beginI2C(*i2cBus_);
    }

    if (startCode > 0) {

        _bme.setMode(MODE_SLEEP);

        delay(10);

        _bme.setHumidityOverSample(1);
        _bme.setPressureOverSample(4);
        _bme.setTempOverSample(1);

        _bme.setFilter(4);

        _bme.setStandbyTime(0);

        _bme.setMode(MODE_NORMAL);

        _lastMeasurement = micros();
        

        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("BME280 Start Fail. Code: " + String(startCode));
    }

    _startAttempts++;

    if (_startAttempts >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
