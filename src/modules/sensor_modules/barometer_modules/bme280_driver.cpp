#include "KraftKontrol/modules/sensor_modules/barometer_modules/bme280_driver.h"



void BME280Driver::getData() {

    BME280_SensorMeasurements measurements;

    int64_t _newDataTimestamp = NOW();
    _bme.readAllMeasurements(&measurements);

    float bufMeasurement = measurements.pressure;
    if (bufMeasurement > 100) {
        DataTimestamped<float> value(bufMeasurement, _newDataTimestamp);
        baroTopic_.publish(value);
        _lastPressure = bufMeasurement;
    }

    bufMeasurement = measurements.temperature;
    if (true) {
        _temperatureFifo.placeFront(bufMeasurement, true);
        _temperatureTimestampFifo.placeFront(_newDataTimestamp, true);
        _lastTemperature = bufMeasurement;
    }

    bufMeasurement = measurements.humidity;
    if (true) {
        _humidityFifo.placeFront(bufMeasurement, true);
        _humidityTimestampFifo.placeFront(_newDataTimestamp, true);
        _lastHumidity = bufMeasurement;
    }

}


void BME280Driver::thread() {

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
        suspendUntil(END_OF_TIME);

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
        _bme.setPressureOverSample(16);
        _bme.setTempOverSample(1);

        _bme.setFilter(16);

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
