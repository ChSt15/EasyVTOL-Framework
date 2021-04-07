#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H



#include "definitions.h"

#include "interval_control.h"

#include "barometer_template.h"

#include "SparkFunBME280.h"
#include "CircularBuffer.h"



class BME280Driver: public BarometerTemplate {
public:

    
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
    bool pressureAvailable() {return !_pressureFifo.isEmpty();};

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

        if (_pressureFifo.isEmpty()) return false;

        *pressureData = _pressureFifo.pop();
        *pressureTimestamp = _pressureTimestampFifo.pop();

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

        if (_pressureFifo.isEmpty()) return false;

        *pressureData = _pressureFifo.last();
        *pressureTimestamp = _pressureTimestampFifo.last();

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
    bool temperatureAvailable() {return !_temperatureFifo.isEmpty();};

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

        if (_temperatureFifo.isEmpty()) return false;

        *temperatureData = _temperatureFifo.pop();
        *temperatureTimestamp = _temperatureTimestampFifo.pop();

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

        if (_temperatureFifo.isEmpty()) return false;

        *temperatureData = _temperatureFifo.last();
        *temperatureTimestamp = _temperatureTimestampFifo.last();

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
    bool humidityAvailable() {return !_humidityFifo.isEmpty();};

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

        if (_humidityFifo.isEmpty()) return false;

        *humidityData = _humidityFifo.pop();
        *humidityTimestamp = _humidityTimestampFifo.pop();

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

        if (_humidityFifo.isEmpty()) return false;

        *humidityData = _humidityFifo.last();
        *humidityTimestamp = _humidityTimestampFifo.last();

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


    CircularBuffer <float, 100> _pressureFifo;
    CircularBuffer <float, 100> _humidityFifo;
    CircularBuffer <float, 100> _temperatureFifo;
    CircularBuffer <uint32_t, 100> _pressureTimestampFifo;
    CircularBuffer <uint32_t, 100> _humidityTimestampFifo;
    CircularBuffer <uint32_t, 100> _temperatureTimestampFifo;

    float _lastPressure;
    float _lastHumidity;
    float _lastTemperature;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

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


extern BME280Driver Baro;





#endif