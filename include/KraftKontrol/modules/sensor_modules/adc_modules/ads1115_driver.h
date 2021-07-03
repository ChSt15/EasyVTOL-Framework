#ifndef ADS1115_DRIVER_H
#define ADS1115_DRIVER_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "adc_interface.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/ADS1X15-master/ADS1X15.h"

#include "KraftKontrol/utils/buffer.h"



class ADS1115Driver: public ADC_Interface, public Module_Abstract, public Task_Abstract {
public:

    ADS1115Driver(TwoWire* i2cBus) : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_Realtime, true), adc_(0x48) {
        i2cBus_ = i2cBus;
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
    uint32_t loopRate() {return loopRate_;};

    /**
     * Returns true if pressure data available
     *
     * @param channel which channel to check if data available
     * @return bool.
     */
    virtual uint32_t voltageAvailable(uint8_t channel = 0) {return voltageFifo_[min(channel, (uint8_t)4)].available();}

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t measurementRate(uint8_t channel = 0) {return adcRate_;}

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param voltageData float where the data will be written into
     * @param voltageTimestamp timestamp in microseconds of when measurement was taken
     * @param channel Which adc channel to get the voltage from
     * @return bool.
     */
    virtual bool getVoltage(float* voltageData, uint32_t* voltageTimestamp, uint8_t channel = 0) {

        channel = min(channel, (uint8_t)4); 

        if (!voltageFifo_[channel].available()) return false;

        voltageFifo_[channel].takeBack(voltageData);
        voltageTimestampFifo_[channel].takeBack(voltageTimestamp);

        return true;

    };

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param voltageData float where the data will be written into
     * @param voltageTimestamp timestamp in microseconds of when measurement was taken
     * @return bool.
     */
    virtual bool peekVoltage(float* voltageData, uint32_t* voltageTimestamp, uint8_t channel) {

        channel = min(channel, (uint8_t)4); 

        if (voltageFifo_[channel].available() == 0) return false;

        voltageFifo_[channel].peekBack(voltageData);
        voltageTimestampFifo_[channel].peekBack(voltageTimestamp);

        return true;

    };

    /**
     * Removes all values from buffer
     *
     * @param channel Which adc channel to flush data
     * @return bool.
     */
    virtual void flushVoltage(uint8_t channel = 0) {
        voltageFifo_[min(channel, (uint8_t)4)].clear();
        voltageTimestampFifo_[min(channel, (uint8_t)4)].clear();
    }


private:

    void _getData();


    Buffer <float, 10> voltageFifo_[4];
    Buffer <uint32_t, 10> voltageTimestampFifo_[4];

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    int chipSelectPin_ = 0;
    TwoWire* i2cBus_;
    ADS1115 adc_;

    uint8_t currentPin_ = 0;

    uint8_t startAttempts_ = 0;

    uint32_t loopRate_ = 0;
    uint32_t loopCounter_ = 0;

    uint32_t adcRate_ = 0;
    uint32_t adcCounter_ = 0;

    uint32_t lastMeasurement_ = 0;

    bool block_ = false;



    
};





#endif