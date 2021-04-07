#include "bme280_driver.h"



BME280Driver Baro; //For outside use



void BME280Driver::_getData() {

    BME280_SensorMeasurements measurements;

    uint32_t _newDataTimestamp = micros();
    _bme.readAllMeasurements(&measurements);

    float bufMeasurement = measurements.pressure;
    if (true) {
        _pressureFifo.unshift(bufMeasurement);
        _pressureTimestampFifo.unshift(_newDataTimestamp);
        _lastPressure = bufMeasurement;
        _pressureCounter++;
    }

    bufMeasurement = measurements.temperature;
    if (true) {
        _temperatureFifo.unshift(bufMeasurement);
        _temperatureTimestampFifo.unshift(_newDataTimestamp);
        _lastTemperature = bufMeasurement;
        _temperatureCounter++;
    }

    bufMeasurement = measurements.humidity;
    if (true) {
        _humidityFifo.unshift(bufMeasurement);
        _humidityTimestampFifo.unshift(_newDataTimestamp);
        _lastHumidity = bufMeasurement;
        _humidityCounter++;
    }

}


void BME280Driver::thread() {

    if (_block) return;

    _loopCounter++;


    if (_moduleStatus == MODULE_STATUS::MODULE_RUNNING) {

        if (!_bme.isMeasuring()) {
            
            _getData();

        }

    } else if (_moduleStatus == MODULE_STATUS::MODULE_NOT_STARTED || _moduleStatus == MODULE_STATUS::MODULE_RESTARTATTEMPT) {
        
        //Serial.println("Test1");
        int startCode = _bme.beginSPI(BME280_NCS_PIN);
        //Serial.println("Test2");


        if (startCode > 0) {

            _bme.setHumidityOverSample(1);
            _bme.setPressureOverSample(4);
            _bme.setTempOverSample(1);

            _bme.setFilter(4);

            _bme.setStandbyTime(0);

            _bme.setMode(MODE_NORMAL);

            _lastMeasurement = micros();
            

            //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
            _moduleStatus = MODULE_STATUS::MODULE_RUNNING;

        } else {
            _moduleStatus = MODULE_STATUS::MODULE_RESTARTATTEMPT; 
            Serial.println("NEW! IMU Start Fail. Code: " + String(startCode));
            delay(1000);
        }

        _startAttempts++;

        if (_startAttempts >= 5 && _moduleStatus == MODULE_STATUS::MODULE_RESTARTATTEMPT) _moduleStatus = MODULE_STATUS::MODULE_FAILURE;

    } else if (false/*_moduleStatus == MODULE_STATUS::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        _moduleStatus = MODULE_STATUS::MODULE_RUNNING; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        _moduleStatus = MODULE_STATUS::MODULE_FAILURE;
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



}
