#ifndef BAROMETER_INTERFACE_H
#define BAROMETER_INTERFACE_H



#include "stdint.h"



class Barometer_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns true if pressure data available
     *
     * @return bool.
     */
    virtual bool pressureAvailable() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t pressureRate() = 0;

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param pressureData float where the data will be writen into
     * @param pressureTimestamp timestamp of when the measurement was taken
     * @return bool.
     */
    virtual bool getPressure(float* pressureData, uint32_t* pressureTimestamp) = 0;

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     * 
     * @see getPressure(...)
     *
     * @param pressureData float where the data will be writen into
     * @param pressureTimestamp timestamp of when the measurement was taken
     * @return bool.
     */
    virtual bool peekPressure(float* pressureData, uint32_t* pressureTimestamp) = 0;

    /**
     * Removes all elements from queue.
     *
     * @return none.
     */
    virtual void flushPressure() = 0;

private:



    
};





#endif