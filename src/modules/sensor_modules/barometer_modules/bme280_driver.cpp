#include "bme280_driver.h"



void BME280Driver::_getData() {

    BME280_SensorMeasurements measurements;

    uint32_t _newDataTimestamp = micros();
    _bme.readAllMeasurements(&measurements);

    float bufMeasurement = measurements.pressure;
    if (true) {
        _pressureFifo.push_front(bufMeasurement);
        _pressureTimestampFifo.push_front(_newDataTimestamp);
        _lastPressure = bufMeasurement;
        _pressureCounter++;
    }

    bufMeasurement = measurements.temperature;
    if (true) {
        _temperatureFifo.push_front(bufMeasurement);
        _temperatureTimestampFifo.push_front(_newDataTimestamp);
        _lastTemperature = bufMeasurement;
        _temperatureCounter++;
    }

    bufMeasurement = measurements.humidity;
    if (true) {
        _humidityFifo.push_front(bufMeasurement);
        _humidityTimestampFifo.push_front(_newDataTimestamp);
        _lastHumidity = bufMeasurement;
        _humidityCounter++;
    }

}


void BME280Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        if (!_bme.isMeasuring()) {
            
            _getData();

        }

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

    if (useSPI_) startCode = _bme.beginSPI(chipSelectPin_);
    else startCode = _bme.beginI2C(*i2cBus_);

    if (startCode > 0) {

        _bme.setHumidityOverSample(1);
        _bme.setPressureOverSample(4);
        _bme.setTempOverSample(1);

        _bme.setFilter(4);

        _bme.setStandbyTime(0);

        _bme.setMode(MODE_FORCED);

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
