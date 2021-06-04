#ifndef EEPROM_HAL_INTERFACE_H
#define EEPROM_HAL_INTERFACE_H



#include "stdint.h"

#include "KraftPacket_KontrolPackets/kraftkontrol_message_types.h"

//#include "Arduino.h"



class EEPROM_Interface {
public:

    EEPROM_Interface(const uint32_t &version) {
        version_ = version;
    }

    /**
     * Writes new version to EEPROM
     * @param version Pointer to version container for receiving version.
     * @param size Pointer to version container for receiving number of bytes the EEPROM is.
     * @returns true if successfull.
     */
    bool readEEPROMData(uint32_t* version, uint32_t* size);

    /**
     * Writes new version to EEPROM
     * @param version Integer of version.
     * @returns true if successfull.
     */
    bool updateVersion(const uint32_t &version);

    /**
     * Writes a message at the given address.
     * You must call commitChanges() to actually save changed to EEPROM memory. 
     * If writting lots of things fast then use this and call commitChanges() only once after all calls are finished.
     * @see commitChanges()
     * @param message Pointer to message to write.
     * @param address Address to write message to.
     * @returns true if successful.
     */
    bool writeMessage(const KraftMessage_Interface* message, const uint32_t &address);

    /**
     * Reads a message from given address.
     * @param message Pointer to message to receive message
     * @param address Address where to read from.
     * @returns true if successful.
     */
    bool readMessage(KraftMessage_Interface* message, const uint32_t &address);

    /**
     * Writes only changed values to EEPROM. This to to reduce wear on EEPROM
     * @returns true if successful
     */
    virtual bool commitChanges() = 0;

protected:

    virtual bool readBytes(const uint32_t &address, uint8_t* data, const uint32_t &numberBytes) = 0;

    virtual bool writeBytes(const uint32_t &address, const uint8_t* data, const uint32_t &numberBytes) = 0;


private:

    uint32_t version_ = 0;

    
};





#endif