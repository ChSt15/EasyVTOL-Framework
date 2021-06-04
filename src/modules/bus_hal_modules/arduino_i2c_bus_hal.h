#ifndef ARDUINO_I2C_BUS_HAL_H
#define ARDUINO_I2C_BUS_HAL_H



#include "Wire.h"



class I2CBus_HAL {
protected:

    I2CBus_HAL(TwoWire* bus) {
        bus_ = bus;
    }

    bool initBus(const int32_t &speed = -1, const int16_t &scl = -1, const int16_t &sda = -1, const int16_t &address = -1) {

        if (address != -1) bus_->begin(address);
        else bus_->begin();

        if (speed != -1) bus_->setClock(speed);

        if (scl != -1) bus_->setSCL(scl);
        if (sda != -1) bus_->setSDA(sda);

    }

    //Write functions

    inline bool writeByte(const uint8_t &address, const uint8_t &reg, const uint8_t &data) {
        return writeBytes(address, reg, &data, 1);
    }

    /**
     * 
     * 
     */
    inline bool writeBytes(const uint8_t &address, const uint8_t &reg, const void* data, const uint32_t numberBytes) {

        bus_->beginTransmission(address);

        bus_->write(reg);
        bus_->write((uint8_t*)data, numberBytes);

        return bus_->endTransmission() == 0;

    }


    //Read functions

    inline bool readByte(const uint8_t &address, const uint8_t &reg, void* data) {
        return readBytes(address, reg, data, 1);
    }

    inline bool readBytes(const uint8_t &address, const uint8_t &reg, void* data, const uint32_t numberBytes) {

        bus_->beginTransmission(address);
        bus_->write(reg);
        if (bus_->endTransmission(false) != 0) {
            return false;
        }

        if (bus_->requestFrom(address, numberBytes) != numberBytes) {
            return false;
        }

        for (uint32_t i = 0; i < numberBytes; i++) ((uint8_t*)data)[i] = bus_->read();

        return true;

    }


private:

    TwoWire* bus_;

    
};





#endif