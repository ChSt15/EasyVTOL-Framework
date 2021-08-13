#ifndef BUS_DEVICE_HAL_H
#define BUS_DEVICE_HAL_H


#include "bus_hal_abstract.h"

#include "stdint.h"


class BusDevice_HAL {
public:

    /**
     * @param bus Bus that will be used for communication
     * @param selectAddress Pin(SPI) or address(I2C, CAN, etc.) used for communication.
     */
    BusDevice_HAL(Bus_HAL_Abstract& bus, uint32_t selectAddress, uint32_t speed, bool invertChipSelect): bus_(bus) {
        selectAddress_ = selectAddress;
    }

    /**
     * First Writes bytes to a starting register, then reads bytes from a starting register
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param readRegister Register to start reading from.
     * @param readData Pointer to where read data will be stored.
     * @param numBytesRead Number of bytes to read. Defaults to 1.
     * @param numBytesWrite Number of bytes to write. Defaults to 1.
     * @returns true if successfull.
     */
    bool writeRead(uint32_t writeRegister, const void* writeData, uint32_t readRegister, void* readData, uint32_t numBytesRead = 1, uint32_t numBytesWrite = 1) {
        
        if (!writeBytes(writeRegister, writeData, numBytesWrite, false)) {
            release(); //Must release device from device from bus before leaving!
            return false;
        }

        if (!writeBytes(readRegister, readData, numBytesRead, true)) {
            return false;
        }

        return true;

    }

    /**
     * Writes a byte to a register.
     * @param writeRegister Register to write to.
     * @param writeData Data to be written to device.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool writeByte(uint32_t writeRegister, uint8_t writeData, bool release = true) {
        return writeBytes(writeRegister, &writeData, 1, release);
    }


    /**
     * Reads a byte from a register.
     * @param readRegister Register to read from
     * @param readData Reference to byte to store data.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool readByte(uint32_t readRegister, uint8_t& readData, bool release = true) {
        return writeBytes(readRegister, &readData, 1, release);
    }

    /**
     * All following functions must be implemented!
     */

    /**
     * Reads bytes from a starting register.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    virtual bool readBytes(uint32_t readRegister, void* readData, uint32_t numberBytes, bool release = true) {
        bool success = true;
        bus_.selectDevice(selectAddress_);
        success = success && bus_.writeBytes(&readRegister, 1);
        success = success && bus_.readBytes(readData, numberBytes);
        if (release) bus_.deselectDevice();
        return success;
    }

    /**
     * Writes bytes to a starting register.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    virtual bool writeBytes(uint32_t writeRegister, const void* writeData, uint32_t numberBytes, bool release = true) {
        bool success = true;
        bus_.selectDevice(selectAddress_);
        success = success && bus_.writeBytes(&writeRegister, 1);
        success = success && bus_.writeBytes(writeData, numberBytes);
        if (release) bus_.deselectDevice();
        return success;
    }

    /**
     * Dependant on implementation of bus. Might have no effect.
     * Stops communication on bus.
     */
    void release() {
        bus_.deselectDevice();
    };

    /**
     * Sets the device address on Bus. This could be a pin for SPI or address for I2C.
     * @param deviceAddress
     */
    void setAddress(uint32_t selectAddress) {
        selectAddress_ = selectAddress;
    }

    /**
     * @see setAddress(uint32_t address)
     * @returns device address
     */
    uint32_t getAddress() const {
        return selectAddress_;
    }

    /**
     * Sets the bus to use for communication.
     * @param bus Bus to use
     */
    void setBus(Bus_HAL_Abstract& bus) {
        bus = bus_;
    }

    /**
     * @returns device bus
     */
    const Bus_HAL_Abstract& getBus() const {
        return bus_;
    }


private:

    Bus_HAL_Abstract& bus_;

    uint32_t selectAddress_ = 0;

    
};





#endif