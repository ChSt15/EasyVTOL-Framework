#ifndef GNSS_INTERFACE_H
#define GNSS_INTERFACE_H



#include "stdint.h"



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
     * @returns true if horizontal position available.
     */
    virtual bool positionAvailable() = 0;

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
     * @param latitude is the latitude data and is a double. Unit is in radians.
     * @param longitude is the longitude data and is a double. Unit is in radians.
     * @returns true if position data valid.
     */
    virtual bool getPosition(double* latitude, double* longitude, uint32_t* positionTimestamp) = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param latitude is the latitude data and is a double. Unit is in radians.
     * @param longitude is the longitude data and is a double. Unit is in radians.
     * @returns true if position data valid.
     */
    virtual bool peekPosition(double* latitude, double* longitude, uint32_t* positionTimestamp) = 0;

    /**
     * @returns the Position accuracy. If unsupported will return -1;
     */
    virtual float getPositionAccuracy() {return -1;}

    /**
     * Removes all elements from queue.
     */
    virtual void flushPosition() = 0;

    /**
     * This is only for horizontal position. Even if this is true then that does not mean height is available
     *
     * @returns true if horizontal position available.
     */
    virtual bool velocityAvailable() = 0;

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
     * @param north is the velocity in north direction
     * @param east is the velocity in east direction
     * @param down is the velocity in down direction
     * @returns true if position data valid.
     */
    virtual bool getVelocity(float* north, float* east, float* down, uint32_t* velocityTimestamp) = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param north is the velocity in north direction
     * @param east is the velocity in east direction
     * @param down is the velocity in down direction
     * @returns true if position data valid.
     */
    virtual bool peekVelocity(float* north, float* east, float* down, uint32_t* velocityTimestamp) = 0;

    /**
     * Removes all elements from queue.
     */
    virtual void flushVelocity() = 0;

    /**
     * Checks if altitude data available.
     *
     * @returns true if altitude data available.
     */
    virtual bool altitudeAvailable() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    virtual uint32_t altitudeRate() = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param altitude is the latitude data.
     * @returns true if altitude data valid.
     */
    virtual bool getAltitude(float* altitude, uint32_t* altitudeTimestamp) = 0;

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param altitude is the latitude data.
     * @returns true if altitude data valid.
     */
    virtual bool peekAltitude(float* altitude, uint32_t* altitudeTimestamp) = 0;

    /**
     * @returns the altitude accuracy. If unsupported will return -1;
     */
    virtual float getAltitudeAccuracy() {return -1;}

    /**
     * Removes all elements from queue.
     */
    virtual void flushAltitude() = 0;

    /**
     * @returns the number of satellites used.
     */
    virtual void numberSatellites() = 0;

    
};





#endif