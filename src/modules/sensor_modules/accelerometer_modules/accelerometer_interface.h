#ifndef ACCELEROMETER_INTERFACE_H
#define ACCELEROMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"



class Accelerometer_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns true if accel data available
     *
     * @param values none.
     * @return bool.
     */
    virtual uint32_t accelAvailable() = 0;

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t accelRate() = 0;

    /**
     * Returns true if accel data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool getAccel(Vector* accelData, uint32_t* accelTimestamp) = 0;

    /**
     * Returns true if accel data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool peekAccel(Vector* accelData, uint32_t* accelTimestamp) = 0;

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    virtual void flushAccel() = 0;
    
    
};





#endif