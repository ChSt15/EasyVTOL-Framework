#ifndef IMU_TEMPLATE_H
#define IMU_TEMPLATE_H



#include "modules/module_template.h"



class BarometerTemplate: public Module {
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
     * @param values none.
     * @return bool.
     */
    virtual bool pressureAvailable() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t pressureRate() = 0;

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool getPressure(float* pressureData, uint32_t* pressureTimestamp) = 0;

    /**
     * Returns true if pressure data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool peekPressure(float* pressureData, uint32_t* pressureTimestamp) = 0;

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    virtual void flushPressure() = 0;

private:



    
};





#endif