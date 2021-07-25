#ifndef BUS_HAL_ABSTRACT_H
#define BUS_HAL_ABSTRACT_H


#include "stdint.h"


class Bus_HAL_Abstract {
public:

    /**
     * First Writes bytes to a starting register, then reads bytes from a starting register
     * @param deviceAddress Address of device to communicate with.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param readRegister Register to start reading from.
     * @param readData Pointer to where read data will be stored.
     * @param numBytesRead Number of bytes to read. Defaults to 1.
     * @param numBytesWrite Number of bytes to write. Defaults to 1.
     * @returns true if successfull.
     */
    bool writeRead(uint32_t deviceAddress, uint32_t writeRegister, const void* writeData, uint32_t readRegister, void* readData, uint32_t numBytesRead = 1, uint32_t numBytesWrite = 1) {
        
        if (!writeBytes(deviceAddress, writeRegister, writeData, numBytesWrite, false)) {
            release(); //Must release device from device from bus before leaving!
            return false;
        }

        if (!writeBytes(deviceAddress, readRegister, readData, numBytesRead, true)) {
            return false;
        }

        return true;

    }

    /**
     * Writes a byte to a register.
     * @param deviceAddress Address of device to communicate with.
     * @param writeRegister Register to write to.
     * @param writeData Data to be written to device.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool writeByte(uint32_t deviceAddress, uint32_t writeRegister, uint8_t writeData, bool release = true) {
        return writeBytes(deviceAddress, writeRegister, &writeData, 1, release);
    }

    /**
     * Writes bytes to a starting register.
     * @param deviceAddress Address of device to communicate with.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    virtual bool writeBytes(uint32_t deviceAddress, uint32_t writeRegister, const void* writeData, uint32_t numberBytes, bool release = true) = 0;


    /**
     * Reads a byte from a register.
     * @param deviceAddress Address of device to communicate with.
     * @param readRegister Register to read from
     * @param readData Reference to byte to store data.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool readByte(uint32_t deviceAddress, uint32_t readRegister, uint8_t& readData, bool release = true) {
        return writeBytes(deviceAddress, readRegister, &readData, 1, release);
    }

    /**
     * Reads bytes from a starting register.
     * @param deviceAddress Address of device to communicate with.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    virtual bool readBytes(uint32_t deviceAddress, uint32_t readRegister, void* readData, uint32_t numberBytes, bool release = true) = 0;

    /**
     * Dependant on implementation on bus. Might have no effect.
     * Stops communication on bus.
     */
    virtual void release() = 0;

    
};





#endif