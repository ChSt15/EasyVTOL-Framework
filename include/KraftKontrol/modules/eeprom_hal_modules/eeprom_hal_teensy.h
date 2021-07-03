#ifndef EEPROM_HAL_TEENSY_H
#define EEPROM_HAL_TEENSY_H


#ifdef ARDUINO_TEENSY40


#include "Arduino.h"
#include "eeprom_hal_interface.h"
#include "EEPROM.h"



class EEPROM_Teensy: public EEPROM_Interface {
public:

    EEPROM_Teensy(const uint32_t &version): EEPROM_Interface(version) {}

    static void init();

    bool commitChanges();

    bool readBytes(const uint32_t &address, uint8_t* data, const uint32_t &numberBytes);

    bool writeBytes(const uint32_t &address, const uint8_t* data, const uint32_t &numberBytes);


protected:

    

private:

    static bool initialised_;

    static uint8_t eepromState_[1000];
    
    static const uint32_t eepromMax_ = 1000;
    
};


#endif


#endif