#include "KraftKontrol/modules/eeprom_hal_modules/eeprom_hal_interface.h"



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

    uint32_t size = message->getDataSize();

    uint32_t dataID = message->getDataType();
    uint32_t messageID = message->getMessageType();

    uint8_t buffer[size];

    if (!message->getRawData(buffer, size)) {
        //Serial.println("Message Get fail");
        return false;
    }

    uint32_t index = 0;
    writeBytes(address, &dataID, sizeof(dataID));
    index += sizeof(dataID);
    writeBytes(address+index, &messageID, sizeof(messageID));
    index += sizeof(dataID);
    writeBytes(address+index, &size, sizeof(size));
    index += sizeof(dataID);

    if (!writeBytes(address+index, buffer, size)) {
        return false;
    }

    return true;

}



bool EEPROM_Interface::readMessage(KraftMessage_Interface* message, const uint32_t &address) {

    uint8_t size = 0;
    uint8_t id = 0;

    if (!readBytes(address, &id, 1)) {
        //Serial.println("read id failed");
        return false;
    }

    if (!readBytes(address + 1, &size, 1)) {
        //Serial.println("read size failed");
        return false;
    }

    if (size != message->getDataSize() || id != message->getDataType()) {
        //Serial.println(String("message compare failed: id: ") + id + ", size: " + size);
        return false;
    }

    uint8_t buffer[size];

    if (!readBytes(address + 2, buffer, size)) {
        //Serial.println("read bytes failed");
        return false;
    }

    if (!message->setRawData(buffer, size)) {
        //Serial.println("message set failed");
        return false;
    }

    return true;
    
}