#ifndef ARDUINO_I2C_HAL_H
#define ARDUINO_I2C_HAL_H


#include "Wire.h"

#include "../../../KraftKontrol/hal/bus_hal_abstract.h"


class I2CBus_HAL: public Bus_HAL_Abstract {
public:

    I2CBus_HAL (TwoWire& bus) {
        bus_ = &bus;
    }

    bool initBus(uint32_t speed = 0, const int16_t &scl = -1, const int16_t &sda = -1) {
        
        bus_->begin();

        if (speed != 0) bus_->setClock(speed);

        return true;

    }

    /**
     * Writes bytes to a starting register.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @returns true if successfull.
     */
    bool writeBytes(uint32_t writeRegister, const void* writeData, uint32_t numberBytes) {

        bus_->write((uint8_t*)writeData, numberBytes);

        uint32_t sentBytes = bus_->endTransmission(false);

        return sentBytes == numberBytes;

    }


    /**
     * Reads bytes from a starting register.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @returns true if successfull.
     */
    bool readBytes(uint32_t readRegister, void* readData, uint32_t numberBytes) {

        if (bus_->requestFrom((int)address_, (int)numberBytes) != numberBytes) {
            return false;
        }

        for (uint32_t i = 0; i < numberBytes; i++) ((uint8_t*)readData)[i] = bus_->read();

        return true;

    }

    /**
     * Selects the device on given selectAddress. 
     * @param selectAddress Parameter to select device on bus. Could be a pin(SPI) or Address(I2C).
     * @param speed Speed of communication.
     * @param invertChipSelect Only for SPI Chip selection. If true then chip select will be active on low. Default false.
     */
    void selectDevice(uint32_t selectAddress, uint32_t speed, bool invertChipSelect = true) {

        transferBegun_ = true;

        bus_->setClock(speed);
        bus_->beginTransmission((uint8_t)selectAddress);

    }

    /**
     * Deselects whatever device was being communicated with. Needs to be done after communication to free Bus for other devices.
     */
    void deselectDevice() {

        bus_->endTransmission(true);

    }



private:

    uint32_t address_;

    TwoWire* bus_;

    bool transferBegun_ = false;

    
};





#endif 
