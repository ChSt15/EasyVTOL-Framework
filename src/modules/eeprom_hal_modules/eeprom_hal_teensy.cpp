#include "eeprom_hal_teensy.h"

#ifdef ARDUINO_TEENSY40


bool EEPROM_Teensy::initialised_ = false;
uint8_t EEPROM_Teensy::eepromState_[1000];


void EEPROM_Teensy::init() {

    if (initialised_) return;

    EEPROM.begin();

    for (uint32_t i = 0; i < eepromMax_; i++) eepromState_[i] = EEPROM.read(i);

    initialised_ = true;

}


bool EEPROM_Teensy::commitChanges() {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < eepromMax_; i++) {
        if (EEPROM.read(i) != eepromState_[i]) EEPROM.update(i, eepromState_[i]);
    }

    return true;

}


bool EEPROM_Teensy::readBytes(const uint32_t &address, uint8_t* data, const uint32_t &numberBytes) {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < numberBytes; i++) data[i] = eepromState_[address + i];

    return true;

}


bool EEPROM_Teensy::writeBytes(const uint32_t &address, const uint8_t* data, const uint32_t &numberBytes) {

    if (!initialised_) return false;

    for (uint32_t i = 0; i < numberBytes; i++) eepromState_[address + i] =  data[i]; 

    return true;

}


#endif