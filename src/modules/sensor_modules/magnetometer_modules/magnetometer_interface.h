#ifndef MAGNETOMETER_INTERFACE_H
#define MAGNETOMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"



class Magnetometer_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns true if Magnetometer data available
     *
     * @param values none.
     * @return bool.
     */
    virtual uint32_t magAvailable() = 0;

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t magRate() = 0;

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool getMag(Vector* magData, uint32_t* magTimestamp) = 0;

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    virtual bool peekMag(Vector* magData, uint32_t* magTimestamp) = 0;

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    virtual void flushMag() = 0;


private:



    
};





#endif