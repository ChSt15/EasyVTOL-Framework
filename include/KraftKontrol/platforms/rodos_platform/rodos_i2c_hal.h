#ifndef RODOS_I2C_HAL_H
#define RODOS_I2C_HAL_H


#include "KraftKontrol/hal/bus_hal_abstract.h"


class I2CBus_HAL: public Bus_HAL_Abstract {
public:

    I2CBus_HAL (TwoWire& bus) {
        bus_ = &bus;
    }

    bool initBus(const int32_t &speed = -1, const int16_t &scl = -1, const int16_t &sda = -1) {
        
        bus_->begin();

        if (speed != -1) bus_->setClock(speed);

        return true;

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
    bool writeBytes(uint32_t deviceAddress, uint32_t writeRegister, const void* writeData, uint32_t numberBytes, bool release = true) {

        bus_->beginTransmission((int)deviceAddress);

        bus_->write(writeRegister);
        bus_->write((uint8_t*)writeData, numberBytes);

        return bus_->endTransmission() == 0;

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
    bool readBytes(uint32_t deviceAddress, uint32_t readRegister, void* readData, uint32_t numberBytes, bool release = true) {

        bus_->beginTransmission((int)deviceAddress);
        bus_->write(readRegister);
        if (bus_->endTransmission(false) != 0) {
            return false;
        }

        if (bus_->requestFrom((int)deviceAddress, (int)numberBytes) != numberBytes) {
            return false;
        }

        for (uint32_t i = 0; i < numberBytes; i++) ((uint8_t*)readData)[i] = bus_->read();

        return true;

    }

    /**
     * Not implemented yet
     */
    void release() {}


private:

    TwoWire* bus_;

    
};





#endif 
