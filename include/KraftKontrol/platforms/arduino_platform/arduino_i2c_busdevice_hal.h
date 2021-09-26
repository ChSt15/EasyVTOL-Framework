#ifndef ARDUINO_I2C_BUSDEVICE_HAL_H
#define ARDUINO_I2C_BUSDEVICE_HAL_H


#include "Wire.h"

#include "../../../KraftKontrol/hal/bus_device_hal_abstract.h"


/**
 * Class for communication with devices on an I2C bus. 
 * Arduino implementation.
 */
class I2CBusDevice_HAL: public BusDevice_HAL_Abstract {
public:

    /**
     * @param bus TwoWire bus to use.
     * @param address Address of device to communicate with.
     */
    I2CBusDevice_HAL(TwoWire& bus, uint32_t address) {
        bus_ = &bus;
        address_ = address;
    }

    /**
     * Initialises the bus.
     * @param speed Speed of the bus. Defaults to 100kHz.
     * @param scl sclPin of bus. NOT IMPLEMENTED.
     * @param sda sdaPin of bus. NOT IMPLEMENTED.
     * @returns true if successfull.
     */
    bool initBus(uint32_t speed = 100000, int16_t scl = -1, int16_t sda = -1) {
        
        bus_->begin();

        bus_->setClock(speed);

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

        //Serial.println(String("Input register: ") + writeRegister + ", num Bytes: " + numberBytes + ", release: " + release);

        bus_->beginTransmission((uint8_t)address_);

        bus_->write((uint8_t*)&writeRegister, 1);
        bus_->write((uint8_t*)writeData, numberBytes);

        return bus_->endTransmission(release) == 0; // Should return 0 if successfull.

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

        bus_->beginTransmission((uint8_t)address_);
        bus_->write(readRegister);
        bus_->endTransmission(false);

        if (bus_->requestFrom((int)address_, (int)numberBytes, (int)release) != numberBytes) {
            return false;
        }

        uint32_t i;
        for (i = 0; i < numberBytes; i++) ((uint8_t*)readData)[i] = bus_->read();

        return i == numberBytes;

    }

    /**
     * Stops communication on bus. Must be done if release on transmition was set to false.
     */
    void release() override {
        bus_->endTransmission(true);
    }


private:

    uint32_t address_;

    TwoWire* bus_;

    //bool transferBegun_ = false;

    
};





#endif 
