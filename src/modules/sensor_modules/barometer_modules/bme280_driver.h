#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H



#include "Arduino.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "barometer_interface.h"

#include "modules/module_abstract.h"

#include "lib/SparkFun_BME280/src/SparkFunBME280.h"

#include "utils/circular_buffer.h"



class BME280Driver: public Barometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    BME280Driver(int chipSelectPin, SPIClass* spiBus) : Task_Abstract(100, eTaskPriority_t::eTaskPriority_Realtime, true) {
        chipSelectPin_ = chipSelectPin;
        spiBus_ = spiBus;
    }
    
    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t loopRate() {return _loopRate;};

    /**
     * Returns true if pressure data available
     *
     * @param values none.
     * @return bool.
     */
    bool pressureAvailable() {return _pressureFifo.available();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t pressureRate() {return _pressureRate;};

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool getPressure(float* pressureData, uint32_t* pressureTimestamp) {

        if (!_pressureFifo.available()) return false;

        *pressureData = _pressureFifo.pop_back();
        *pressureTimestamp = _pressureTimestampFifo.pop_back();

        return true;

    };

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool peekPressure(float* pressureData, uint32_t* pressureTimestamp) {

        if (!_pressureFifo.available()) return false;

        *pressureData = *_pressureFifo.peek_back();
        *pressureTimestamp = *_pressureTimestampFifo.peek_back();

        return true;

    }

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushPressure() {
        _pressureFifo.clear();
        _pressureTimestampFifo.clear();
    }

    /**
     * Returns true if temperature data available
     *
     * @param values none.
     * @return bool.
     */
    bool temperatureAvailable() {return _temperatureFifo.available();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t temperatureRate() {return _temperatureRate;};

    /**
     * Returns true if temperature data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool getTemperature(float* temperatureData, uint32_t* temperatureTimestamp) {

        if (!_temperatureFifo.available()) return false;

        *temperatureData = _temperatureFifo.pop_back();
        *temperatureTimestamp = _temperatureTimestampFifo.pop_back();

        return true;

    };

    /**
     * Returns true if temperature data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool peekTemperature(float* temperatureData, uint32_t* temperatureTimestamp) {

        if (!_temperatureFifo.available()) return false;

        *temperatureData = *_temperatureFifo.peek_back();
        *temperatureTimestamp = *_temperatureTimestampFifo.peek_back();

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushTemperature() {
        _temperatureFifo.clear();
        _temperatureTimestampFifo.clear();
    }

    /**
     * Returns true if humiditynetometer data available
     *
     * @param values none.
     * @return bool.
     */
    bool humidityAvailable() {return _humidityFifo.available();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t humidityRate() {return _humidityRate;};

    /**
     * Returns true if humiditynetometer data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool getHumidity(float* humidityData, uint32_t* humidityTimestamp) {

        if (!_humidityFifo.available()) return false;

        *humidityData = _humidityFifo.pop_back();
        *humidityTimestamp = _humidityTimestampFifo.pop_back();

        return true;

    };

    /**
     * Returns true if humiditynetometer data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values float and uint32_t.
     * @return bool.
     */
    bool peekHumidity(float* humidityData, uint32_t* humidityTimestamp) {

        if (!_humidityFifo.available()) return false;

        *humidityData = *_humidityFifo.peek_back();
        *humidityTimestamp = *_humidityTimestampFifo.peek_back();

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushHumidity() {
        _temperatureFifo.clear();
        _temperatureTimestampFifo.clear();
    }


private:

    void _getData();


    Circular_Buffer <float, 100> _pressureFifo;
    Circular_Buffer <float, 100> _humidityFifo;
    Circular_Buffer <float, 100> _temperatureFifo;
    Circular_Buffer <uint32_t, 100> _pressureTimestampFifo;
    Circular_Buffer <uint32_t, 100> _humidityTimestampFifo;
    Circular_Buffer <uint32_t, 100> _temperatureTimestampFifo;

    float _lastPressure;
    float _lastHumidity;
    float _lastTemperature;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    int chipSelectPin_ = 0;
    SPIClass* spiBus_;
    BME280 _bme;

    uint8_t _startAttempts = 0;

    uint32_t _loopRate = 0;
    uint32_t _loopCounter = 0;

    uint32_t _pressureRate = 0;
    uint32_t _pressureCounter = 0;

    uint32_t _temperatureRate = 0;
    uint32_t _temperatureCounter = 0;

    uint32_t _humidityRate = 0;
    uint32_t _humidityCounter = 0;

    uint32_t _lastMeasurement = 0;

    bool _block = false;



    
};





#endif