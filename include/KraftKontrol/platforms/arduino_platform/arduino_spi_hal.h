#ifndef ARDUINO_SPI_HAL_H
#define ARDUINO_SPI_HAL_H


#include "SPI.h"

#include "KraftKontrol/hal/bus_hal_abstract.h"


class SPIBus_HAL: public Bus_HAL_Abstract {
protected:

    SPIBus_HAL(SPIClass& bus, const SPISettings &settings, const uint8_t csPin, const bool csPinActiveHigh = false) {
        bus_ = &bus;
        settings_ = settings;
        csPin_ = csPin;
        csPinActiveHigh_ = csPinActiveHigh;
    }

    bool initBus(int32_t sck = -1, int32_t mosi = -1, int32_t miso = -1) {

        bus_->begin();

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

        bus_->beginTransaction(settings_);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        bus_->transfer(writeRegister);
        bus_->transfer((uint8_t*)writeData, numberBytes);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        bus_->endTransaction();

        return true;

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

        bus_->beginTransaction(settings_);

        csPinSet(csPinActiveHigh_);
        #if defined(__IMXRT1062__)
            delayNanoseconds(200);
        #endif

        return true;

    }


    /**
     * Not implemented yet
     */
    void release() {}


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
