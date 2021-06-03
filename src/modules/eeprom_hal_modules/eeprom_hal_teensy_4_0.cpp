#include "eeprom_hal_teensy_4_0.h"


bool EEPROM_Teensy4_0::initialised_ = false;



void EEPROM_Teensy4_0::init() {

    if (initialised_) return;

    EEPROM.begin();

    for (uint32_t i = 0; i < eepromMax_; i++) eepromState_[i] = EEPROM.read(i);

    initialised_ = true;

}


bool EEPROM_Teensy4_0::commitChanges() {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < eepromMax_; i++) {
        if (EEPROM.read(i) != eepromState_[i]) EEPROM.update(i, eepromState_[i]);
    }

    return true;

}


bool EEPROM_Teensy4_0::readBytes(const uint32_t &address, uint8_t* data, const uint32_t &numberBytes) {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < numberBytes; i++) data[i] = eepromState_[address + i];

    return true;

}


bool EEPROM_Teensy4_0::writeBytes(const uint32_t &address, const uint8_t* data, const uint32_t &numberBytes) {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < numberBytes; i++) eepromState_[address + i] =  data[i]; 

    return true;

}