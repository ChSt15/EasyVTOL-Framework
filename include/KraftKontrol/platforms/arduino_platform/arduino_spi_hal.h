#ifndef ARDUINO_SPI_HAL_H
#define ARDUINO_SPI_HAL_H


#include "SPI.h"

#include "../../../KraftKontrol/hal/bus_hal_abstract.h"


class SPIBus_HAL: public Bus_HAL_Abstract {
protected:

    SPIBus_HAL(SPIClass& bus) {
        bus_ = &bus;
    }

    bool initBus(int32_t sck = -1, int32_t mosi = -1, int32_t miso = -1) {

        bus_->begin();

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

        bus_->transfer(writeData, numberBytes);

        return true;

    }

    /**
     * Reads bytes from a starting register.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @returns true if successfull.
     */
    bool readBytes(uint32_t readRegister, void* readData, uint32_t numberBytes) {

        bus_->

        return true;

    }


    /**
     * Selects the device on given selectAddress. 
     * @param selectAddress Parameter to select device on bus. Could be a pin(SPI) or Address(I2C).
     */
    void selectDevice(uint32_t selectAddress, uint32_t speed, bool invertChipSelect = true) {

        transferBegun_ = true;

        selectPin_ = selectAddress;
        selectPinState_ = !invertChipSelect;

        bus_->beginTransaction(settings_);
        digitalWrite(selectAddress, !invertChipSelect);

    }

    /**
     * Deselects whatever device was being communicated with. Needs to be done after communication to free Bus for other devices.
     */
    void deselectDevice() {

        digitalWrite(selectPin_, !selectPinState_);
        bus_->endTransaction();

    }


private:

    SPIClass* bus_;

    uint32_t selectPin_ = 0;
    bool selectPinState_ = false;

    bool transferBegun_ = false;

    SPISettings settings_;

    
};




#endif 
