#include "../../../include/KraftKontrol/platforms/arduino_platform/arduino_data_manager_eeprom.h"


DataManager_InternalEEPROM::DataManager_InternalEEPROM() {

    #ifdef ESP32
    EEPROM.begin(4000);
    #else
    EEPROM.begin();
    #endif

    dataSize_ = EEPROM.length();
    dataPointer_ = new uint8_t[dataSize_]; //create data buffer in RAM.

    loadData(); //Retrieve data from EEPROM. Reading shouldnt decrease EEPROM lifetime.

}


DataManager_InternalEEPROM::~DataManager_InternalEEPROM() {

    dataSize_ = 0;
    delete[] dataPointer_;

}


bool DataManager_InternalEEPROM::readData(uint32_t index, uint8_t* data, uint32_t numBytes) const {
    
    if ((index + numBytes) > dataSize_) return false;

    for (uint32_t i = 0; i < numBytes; i++) data[i] = dataPointer_[index + i];

    return true;

}


bool DataManager_InternalEEPROM::writeData(uint32_t index, const uint8_t* data, uint32_t numBytes) {

    if ((index + numBytes) > dataSize_) return false;

    for (uint32_t i = 0; i < numBytes; i++) dataPointer_[index + i] = data[i];

    return true;

}


uint32_t DataManager_InternalEEPROM::getEndIndex() const {
    return dataSize_-1;
}


bool DataManager_InternalEEPROM::loadData() {

    for (uint32_t i = 0; i < dataSize_; i++) {

        dataPointer_[i] = EEPROM.read(i);

    }

    return true;

}


bool DataManager_InternalEEPROM::saveData() {

    for (uint32_t i = 0; i < dataSize_; i++) {

        uint8_t buf = EEPROM.read(i); //Check EEPROM value.

        //Update only if its different.
        if (buf != dataPointer_[i]) EEPROM.write(i, dataPointer_[i]);

    }

    return true;

}
    
