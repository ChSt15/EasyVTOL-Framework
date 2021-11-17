#ifndef DATA_MANAGER_ABSTRACT_H
#define DATA_MANAGER_ABSTRACT_H



#include "stdint.h"

#include "Arduino.h"

#include "KraftKontrol/modules/communication_modules/kraft_message.h"



/**
 * Class for storing data in an orderly manner. Used with KraftMessages. 
 * 
 * Data byte 0 is version.
 * Internal message structure:
 * 0-3 uint32_t Data size of message without header stuff
 * 4-7 uint32_t next message index
 * 8-11 uint32_t message type
 * 12-15 uint32_t data type
 * 16-19 uint32_t message id
 * 20+ raw data
 */
class DataManager_Abstract {
private:

    ///Current length of data.
    uint32_t currentDataSize_ = 0;


protected:

    ///Storage version 
    const uint8_t storageVersion_ = 0xAE + 1;
    

public:

    /**
     * @param dataSize Number of bytes the storage should be.
     */
    DataManager_Abstract();

    ///Destructor
    virtual ~DataManager_Abstract();

public:

    /**
     * Reads data from data storage.
     * @param index First byte to start reading from.
     * @param data Pointer to data where to write data to.
     * @param numBytes Number of bytes to read.
     * @returns true if successfull.
     */
    virtual bool readData(uint32_t index, uint8_t* data, uint32_t numBytes = 1) const = 0;

    /**
     * Writes data to data storage.
     * @param index First byte to start writting data to.
     * @param data Pointer to data that should be written to data storage.
     * @param numBytes Number of bytes to write.
     * @returns true if successfull.
     */
    virtual bool writeData(uint32_t index, const uint8_t* data, uint32_t numBytes = 1) = 0;

    /**
     * @returns index at end of storage.
     */
    virtual uint32_t getEndIndex() const = 0;

    /**
     * Searches for matching message and id.
     * @param message Message type for search for.
     * @param id Message ID to search for if multiple exist. Defaults to 0. 0 means first message found.
     * @param startIndex Index to start searching from. Must be beginning of a known message. Defaults to 1.
     * @returns index of message data start. Will return 0 if not found.
     */
    uint32_t searchMessage(uint32_t messageTypeSet, uint32_t dataTypeSet, uint32_t id = 0, uint32_t startIndex = 1);

    /**
     * Searches for matching message and id.
     * @param message Message type for search for.
     * @param id Message ID to search for if multiple exist. Defaults to 0. 0 means first message found.
     * @returns index of message data start. Will return 0 if not found.
     */
    //uint32_t searchLastMessage(uint32_t messageTypeSet, uint32_t dataTypeSet, uint32_t id = 0);
    

public:

    /**
     * Searches for message type in data and matching id. And places it into message from data storage.
     * @param message Message type for search for and where to write data to.
     * @param id Message ID to search for if multiple exist. Defaults to 0. 0 means first message found.
     * @returns true if found and copied. False if failure.
     */
    bool getMessage(KraftMessage_Interface& message, uint32_t id = 0);
    
    /**
     * Searches for message type in data and matching id and places data from given message into data storage.
     * @param message Message type for search for and places its data into data storage.
     * @param id Message ID to search for if multiple exist. Defaults to 0. 0 means first message found.
     * @returns true if found and copied. False if failure.
     */
    bool setMessage(KraftMessage_Interface& message, uint32_t id = 0);

    /**
     * Places given message into data storage.
     * @param message Message to be placed into storage.
     * @returns id of message. If 0 then not enough free space available.
     */
    uint32_t newMessage(KraftMessage_Interface& message);

    /**
     * Counts number of messages os type found in storage.
     * @param message Message type to search for.
     * @returns number of messages of type found.
     */
    uint32_t getNumberMessages(KraftMessage_Interface& message);

    /**
     * Clears all data in storage. (Efficient, does not zero all data.)
     */
    void clear();

    /**
     * Searches for message type in data and matching id and removes from storage.
     * @param message Message type to search for.
     * @param id Message ID to search for if multiple exist. Defaults to 0. 0 means first message found.
     * @returns true if found and deleted.
     */
    bool deleteMessage(KraftMessage_Interface& message, uint32_t id = 0);

    /**
     * Deletes all messages of given type.
     * @param message Message type to search for.
     * @returns number of messages found and deleted.
     */
    uint32_t deleteAllMessages(KraftMessage_Interface& message);

    ///Index access operator
    uint8_t operator [] (uint32_t index);


    ///Copy operator.
    //DataManager_Abstract& operator = (DataManager_Abstract& dataStorage); 
    
};



#endif