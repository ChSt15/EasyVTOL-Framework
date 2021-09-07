#ifndef ARDUINO_SPI_BUSDEVICE_HAL_H
#define ARDUINO_SPI_BUSDEVICE_HAL_H


#include "SPI.h"

#include "../../../KraftKontrol/hal/bus_device_hal_abstract.h"


class SPIBusDevice_HAL: public BusDevice_HAL_Abstract {
protected:

    /**
     * @param bus SPI bus to use for communication.
     * @param csPin Chip select pin to select device.
     * @param spiSettings Settings to use to communicate with device.
     * @param invertCs Set to true if to select device, the cs pin must be pulled low. Defaults to true.
     */
    SPIBusDevice_HAL(SPIClass& bus, uint32_t csPin, SPISettings spiSettings, bool invertCs = true) {
        bus_ = &bus;
        csPin_ = csPin;
        invertCs_ = invertCs;
        selectPinState_ = invertCs;
        settings_ = spiSettings;
    }

    /**
     * Initialises the bus.
     * @param sck SCK pin on bus. NOT IMPLEMENTED.
     * @param mosi MOSI Pin on bus. NOT IMPLEMENTED.
     * @param miso MISO Pin on bus. NOT IMPLEMENTED.
     * @returns true if successfull.
     */
    bool initBus(int32_t sck = -1, int32_t mosi = -1, int32_t miso = -1) {

        bus_->begin();

        return true;

    }

    /**
     * Writes bytes to a starting register.
     * @param writeRegister Register to start writing bytes to.
     * @param writeData Pointer to Data that will be written.
     * @param numberBytes Number of bytes from writeData to write.
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool writeBytes(uint32_t writeRegister, const void* writeData, uint32_t numberBytes, bool release = true) override {

        //Needed because arduino spi function does not support const void* parameter.      ):
        uint8_t buffer[numberBytes];
        memcpy(buffer, writeData, numberBytes);

        bus_->beginTransaction(settings_);
        selectDevice();

        bus_->transfer(writeRegister | writeCommand_);
        bus_->transfer(buffer, numberBytes);

        if (release) {
            deselectDevice();
            bus_->endTransaction();
        }

        return true;

    }


    /**
     * Reads bytes from a starting register.
     * @param readRegister Register to start reading bytes from.
     * @param readData Pointer to where to store data.
     * @param numberBytes Number of bytes to read from device
     * @param release Unselect the device from bus. If false then call after transfer release(). Depending on implementation can optimise transfer. Defaults to true.
     * @returns true if successfull.
     */
    bool readBytes(uint32_t readRegister, void* readData, uint32_t numberBytes, bool release = true) override {

        bus_->beginTransaction(settings_);
        selectDevice();

        bus_->transfer(readRegister | readCommand_);
        bus_->transfer((uint8_t*)readData, numberBytes);

        if (release) {
            deselectDevice();
            bus_->endTransaction();
        }

        return true;

    }

    /**
     * Stops communication on bus. Must be done if release on transmition was set to false.
     */
    void release() override {
        deselectDevice();
        bus_->endTransaction();
    }

    /**
     * Changes to command used to read data from device.
     * @param command Command to use. Defualts to standard 0x80.
     */
    void setReadCommand(uint8_t command = 0x80) {
        readCommand_ = command;
    }

    /**
     * @returns command used to read data from device.
     */
    uint8_t getReadCommand() const {return readCommand_;}

    /**
     * Changes to command used to write data to device.
     * @param command Command to use. Defualts to standard 0x00.
     */
    void setWriteCommand(uint8_t command = 0x00) {
        writeCommand_ = command;
    }

    /**
     * @returns command used to write data to device.
     */
    uint8_t getWriteCommand() const {return writeCommand_;}


private:

    /**
     * Selects the device.
     */
    void selectDevice() {

        digitalWrite(csPin_, !invertCs_);

    }

    /**
     * Deselects whatever device was being communicated with. Needs to be done after communication to free Bus for other devices.
     */
    void deselectDevice() {

        digitalWrite(csPin_, invertCs_);
        bus_->endTransaction();

    }

    SPIClass* bus_;

    uint32_t csPin_ = 0;
    bool invertCs_ = false;
    bool selectPinState_ = false;

    //bool transferBegun_ = false;

    SPISettings settings_;

    //Command to read data from device.
    uint8_t readCommand_ = 0x80;

    //Command to write data to device.
    uint8_t writeCommand_ = 0x00;

};




#endif 
