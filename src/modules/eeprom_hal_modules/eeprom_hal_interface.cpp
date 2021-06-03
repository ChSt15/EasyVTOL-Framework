#include "eeprom_hal_interface.h"



bool EEPROM_Interface::readEEPROMData(uint32_t* version, uint32_t* size) {

    uint8_t buffer[4]; 

    //Read version
    if (!readBytes(0, buffer, 4)) return false;

    *version = (uint32_t) buffer[0] | buffer[1]<<8 | buffer[2]<<16 | buffer[3]<<24;

    if (!readBytes(4, buffer, 4)) return false;

    *size = (uint32_t) buffer[0] | buffer[1]<<8 | buffer[2]<<16 | buffer[3]<<24;

    return true;

}



bool EEPROM_Interface::updateVersion(const uint32_t &version) {

    uint8_t buffer[4]; 

    //Convert to buffer array
    buffer[0] = version;
    buffer[1] = version>>8;
    buffer[2] = version>>16;
    buffer[3] = version>>24;

    //Write version
    if (!readBytes(0, buffer, 4)) return false;

    return true;

}



bool EEPROM_Interface::writeMessage(const KraftMessage_Interface* message, const uint32_t &address) {

    uint8_t size = message->getDataSize();

    uint8_t id = message->getDataTypeID();

    uint8_t buffer[size + 2];

    buffer[0] = id;
    buffer[1] = size;

    if (!message->getRawData(buffer, size + 2, 2)) return false;

    if (!writeBytes(address, buffer, size + 2)) return false;

    return true;

}



bool EEPROM_Interface::readMessage(KraftMessage_Interface* message, const uint32_t &address) {

    uint8_t size = 0;
    uint8_t id = 0;

    if (!readBytes(address, &size, 1)) return false;

    if (!readBytes(address + 1, &id, 1)) return false;

    if (size != message->getDataSize() || id != message->getDataTypeID()) return false;

    uint8_t buffer[size];

    if (!readBytes(address + 2, buffer, size)) return false;

    message->setRawData(buffer, size);

    return true;
    
}