#ifndef ARDUINO_SPI_BUS_HAL_H
#define ARDUINO_SPI_BUS_HAL_H



#include "SPI.h"



class SPIBus_HAL {
protected:

    SPIBus_HAL(SPIClass* bus, const SPISettings &settings, const uint8_t csPin, const bool csPinActiveHigh = false) {
        bus_ = bus;
        settings_ = settings;
        csPin_ = csPin;
        csPinActiveHigh_ = csPinActiveHigh;
    }

    bool initBus(const int16_t &sck = -1, const int16_t &mosi -1, const int16_t &miso = -1) {

        bus_->begin();

    }

    //Write functions

    inline bool writeByte(const uint8_t &reg, const uint8_t &data) {
        return writeBytes(reg, &data, 1);
    }

    /**
     * 
     * 
     */
    inline bool writeBytes(const uint8_t &reg, const void* data, const uint32_t numberBytes) {

        bus_->beginTransaction(settings_);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        bus_->transfer(reg);
        bus_->transfer((void*)data, numberBytes);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        bus_->endTransaction();

        return true;

    }


    //Read functions

    inline bool readByte(const uint8_t &reg, void* data) {
        return readBytes(reg, data, 1);
    }

    inline bool readBytes(const uint8_t &reg, void* data, const uint32_t numberBytes) {

        bus_->beginTransaction(settings_);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        return true;

    }


private:

    inline void csPinSet(const bool &output) {
        #if defined(__MK20DX128__) || defined(__MK20DX256__) || \
        defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
        defined(__MKL26Z64__)  || defined(__IMXRT1062__) || \
        defined(__IMXRT1052__)
            digitalWriteFast(csPin_, output);
        #else
            digitalWrite(csPin_, output);
        #endif
    }

    SPIClass* bus_;

    uint8_t csPin_;

    bool csPinActiveHigh_ = false;

    SPISettings settings_;

    
};





#endif