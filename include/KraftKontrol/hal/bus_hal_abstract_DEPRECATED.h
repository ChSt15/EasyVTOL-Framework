#ifndef BUS_HAL_ABSTRACT_H
#define BUS_HAL_ABSTRACT_H


#include "stdint.h"


/**
 * Abstract class used for bus communication. This creates a uniform interface for all bus comms.
 * First select device to communicate with, then do read or write, then you MUST deselect the device.
 * This is not suitable for direct device communication. For that please use BusDevice_HAL.
 */
class Bus_HAL_Abstract {
public:

    /**
     * Selects the device on given selectAddress. 
     * @param selectAddress Parameter to select device on bus. Could be a pin(SPI) or Address(I2C).
     * @param speed Speed of communication.
     * @param invertChipSelect Only for SPI Chip selection. If true then chip select will be active on low. Default false.
     */
    //virtual void selectDevice(uint32_t selectAddress, uint32_t speed, bool invertChipSelect = true) = 0;

    /**
     * Deselects whatever device was being communicated with. Needs to be done after communication to free Bus for other devices.
     */
    //virtual void deselectDevice() = 0;

    /**
     * Writes bytes to a starting register.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @returns true if successfull.
     */
    virtual bool writeBytes(const void* writeData, uint32_t numberBytes) = 0;

    /**
     * Reads bytes from a starting register.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @returns true if successfull.
     */
    virtual bool readBytes(void* readData, uint32_t numberBytes) = 0;

    
};





#endif