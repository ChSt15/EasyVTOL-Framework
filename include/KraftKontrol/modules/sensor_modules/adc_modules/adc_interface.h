#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H



#include "stdint.h"



class ADC_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns true if pressure data available
     *
     * @param channel which channel to check if data available
     * @return bool.
     */
    virtual uint32_t voltageAvailable(uint8_t channel = 0) = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @param channel which channel to check the rate
     */
    virtual uint32_t measurementRate(uint8_t channel = 0) = 0;

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
    virtual bool getVoltage(float* voltageData, uint32_t* voltageTimestamp, uint8_t channel = 0) = 0;

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
    virtual bool peekVoltage(float* voltageData, uint32_t* voltageTimestamp, uint8_t channel) = 0;

    /**
     * Removes all values from buffer
     *
     * @param channel Which adc channel to flush data
     * @return bool.
     */
    virtual void flushVoltage(uint8_t channel = 0) = 0;

private:



    
};





#endif