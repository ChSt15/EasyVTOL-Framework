#ifndef GYROSCOPE_INTERFACE_H
#define GYROSCOPE_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"



class Gyroscope_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns true if gyro data available
     *
     * @param values none.
     * @return bool.
     */
    virtual bool gyroAvailable() = 0;

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t gyroRate() = 0;

    /**
     * Returns true if gyro data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool getGyro(Vector* gyroData, uint32_t* gyroTimestamp) = 0;

    /**
     * Returns true if gyro data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool peekGyro(Vector* gyroData, uint32_t* gyroTimestamp) = 0;

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    virtual void flushGyro() = 0;

    
};





#endif