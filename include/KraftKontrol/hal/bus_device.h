#ifndef BUS_DEVICE_H
#define BUS_DEVICE_H



#include "stdint.h"
#include "bus_hal_abstract.h"



/**
 * Class used to make communicating with devices on busses easier.
 */
class BusDevice {
public:

    BusDevice(uint32_t deviceAddress, Bus_HAL_Abstract& bus): deviceBus_(bus) {
        deviceAddress_ = deviceAddress;
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
    inline bool writeRead(uint32_t writeRegister, const void* writeData, uint32_t readRegister, void* readData, uint32_t numBytesRead = 1, uint32_t numBytesWrite = 1) {
        return deviceBus_.writeRead(deviceAddress_, writeRegister, writeData, readRegister, readData, numBytesRead, numBytesWrite);
    }

    /**
     * Writes a byte to a register.
     * @param writeRegister Register to write to.
     * @param writeData Data to be written to device.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    inline bool writeByte(uint32_t writeRegister, uint8_t writeData, bool release = true) {
        return deviceBus_.writeBytes(deviceAddress_, writeRegister, &writeData, 1, release);
    }

    /**
     * Writes bytes to a starting register.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    inline bool writeBytes(uint32_t writeRegister, const void* writeData, uint32_t numberBytes, bool release = true) {
        return deviceBus_.writeBytes(deviceAddress_, writeRegister, writeData, numberBytes, release);
    }


    /**
     * Reads a byte from a register.
     * @param readRegister Register to read from
     * @param readData Reference to byte to store data.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    inline bool readByte(uint32_t readRegister, uint8_t& readData, bool release = true) {
        return deviceBus_.readBytes(deviceAddress_, readRegister, &readData, 1, release);
    }

    /**
     * Reads bytes from a starting register.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    inline bool readBytes(uint32_t readRegister, void* readData, uint32_t numberBytes, bool release = true) {
        return deviceBus_.readBytes(deviceAddress_, readRegister, readData, numberBytes, release);
    }

    /**
     * Dependant on implementation on bus. Might have no effect.
     * Stops communication on bus.
     */
    inline void release() {deviceBus_.release();}


protected:

    Bus_HAL_Abstract& deviceBus_;
    uint32_t deviceAddress_;
    
};





#endif