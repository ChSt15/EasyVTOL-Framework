#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "barometer_interface.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/SparkFun_BME280/src/SparkFunBME280.h"

#include "KraftKontrol/utils/buffer.h"



class BME280Driver: public Barometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    BME280Driver(int chipSelectPin, SPIClass* spiBus) : Task_Abstract("BME280 SPI Driver", 50, eTaskPriority_t::eTaskPriority_Realtime) {
        chipSelectPin_ = chipSelectPin;
        spiBus_ = spiBus;
        useSPI_ = true;
    }

    BME280Driver(TwoWire* i2cBus, int address) : Task_Abstract("BME280 I2C Driver", 50, eTaskPriority_t::eTaskPriority_VeryHigh) {
        i2cBus_ = i2cBus;
        i2cAddress_ = address;
        useSPI_ = false;
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

     * @return uint32_t.
     */
    uint32_t loopRate() {return _loopRate;};

    /**
     * Returns rate (in Hz) of the new sensor data

     * @return uint32_t.
     */
    uint32_t pressureRate() {return _pressureRate;};

    /**
     * Returns true if temperature data available
     *
     * @param values none.
     * @return bool.
     */
    uint32_t temperatureAvailable() {return _temperatureFifo.available();};

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
    bool getTemperature(float& temperatureData, uint32_t& temperatureTimestamp) {

        if (_temperatureFifo.available() == 0) return false;

        _temperatureFifo.takeBack(temperatureData);
        _temperatureTimestampFifo.takeBack(temperatureTimestamp);

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
    bool peekTemperature(float& temperatureData, uint32_t& temperatureTimestamp) {

        if (_temperatureFifo.available() == 0) return false;

        _temperatureFifo.peekBack(temperatureData);
        _temperatureTimestampFifo.peekBack(temperatureTimestamp);

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
    uint32_t humidityAvailable() {return _humidityFifo.available();};

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
    bool getHumidity(float& humidityData, uint32_t& humidityTimestamp) {

        if (_humidityFifo.available() == 0) return false;

        _humidityFifo.takeBack(humidityData);
        _humidityTimestampFifo.takeBack(humidityTimestamp);

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
    bool peekHumidity(float& humidityData, uint32_t& humidityTimestamp) {

        if (_humidityFifo.available() == 0) return false;

        _humidityFifo.peekBack(humidityData);
        _humidityTimestampFifo.peekBack(humidityTimestamp);

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

    void getData();


    Buffer <float, 100> _humidityFifo;
    Buffer <float, 100> _temperatureFifo;
    Buffer <uint32_t, 100> _humidityTimestampFifo;
    Buffer <uint32_t, 100> _temperatureTimestampFifo;

    float _lastPressure;
    float _lastHumidity;
    float _lastTemperature;

    IntervalControl _rateCalcInterval = IntervalControl(1); 


    int chipSelectPin_ = 0;
    SPIClass* spiBus_;
    bool useSPI_ = false;

    bool sensorMeasuring_ = false;

    TwoWire* i2cBus_;
    int i2cAddress_;

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