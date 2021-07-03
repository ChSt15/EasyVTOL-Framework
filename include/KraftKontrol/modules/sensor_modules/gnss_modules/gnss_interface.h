#ifndef GNSS_INTERFACE_H
#define GNSS_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/data_containers/navigation_data.h"



class GNSS_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * This is only for horizontal position. Even if this is true then that does not mean height is available
     *
     * @returns number of measurements available in buffer.
     */
    virtual uint16_t positionAvailable() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t positionRate() = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param position Struct to be overritten with position data.
     * @returns true if position data valid.
     */
    virtual bool getPosition(WorldPosition* position, uint32_t* positionTimestamp) = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param position Struct to be overritten with position data.
     * @returns true if position data valid.
     */
    virtual bool peekPosition(WorldPosition* position, uint32_t* positionTimestamp) = 0;

    /**
     * @returns the Position accuracy. If unsupported or altitude not available will return -1;
     */
    virtual float getPositionAccuracy() {return -1;}

    /**
     * @returns the altitude accuracy. If unsupported or altitude not available will return -1;
     */
    virtual float getAltitudeAccuracy() {return -1;}

    /**
     * Removes all elements from queue.
     */
    virtual void flushPosition() = 0;

    /**
     * This is only for horizontal position. Even if this is true then that does not mean height is available
     *
     * @returns number of measurements available in buffer.
     */
    virtual uint16_t velocityAvailable() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t velocityRate() = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param velocity is the velocity.
     * @returns true if position data valid.
     */
    virtual bool getVelocity(Vector<>* velocity, uint32_t* velocityTimestamp) = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param velocity is the velocity.
     * @returns true if position data valid.
     */
    virtual bool peekVelocity(Vector<>* velocity, uint32_t* velocityTimestamp) = 0;

    /**
     * Removes all elements from queue.
     */
    virtual void flushVelocity() = 0;

    /**
     * @returns the number of satellites used.
     */
    virtual uint8_t getNumSatellites() = 0;

    /**
     * @returns true if GNSS lock is valid and safe.
     */
    virtual bool getGNSSLockValid() = 0;
    
};





#endif