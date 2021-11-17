#include "KraftKontrol/modules/data_manager_modules/data_manager_abstract.h"



DataManager_Abstract::DataManager_Abstract() {}


DataManager_Abstract::~DataManager_Abstract() {}



/*bool DataManager_Abstract::readData(uint32_t index, uint8_t* data, uint32_t numBytes) const {

    //Make sure we wont read past the end of array.
    if ((index + numBytes - 1) >= maxDataSize_) return false;

    for (uint32_t i = 0; i < numBytes; i++) data[i] = dataPointer_[index + i];

    return true;

}


bool DataManager_Abstract::writeData(uint32_t index, const uint8_t* data, uint32_t numBytes) {

    //Make sure we wont write past the end of array.
    if ((index + numBytes - 1) >= maxDataSize_) return false;

    for (uint32_t i = 0; i < numBytes; i++) dataPointer_[index + i] = data[i];

    return true;

}*/


uint32_t DataManager_Abstract::searchMessage(uint32_t messageTypeSet, uint32_t dataTypeSet, uint32_t id, uint32_t startIndex) {

    uint32_t index = startIndex;

    uint32_t messageType = 0;
    uint32_t dataType = 0;
    uint32_t messageID = 0;

    bool exit = false;

    while (!exit) {

        ///get message type and data type
        if (!readData(index + 8, (uint8_t*)&messageType, 4)) break;
        if (!readData(index + 12, (uint8_t*)&dataType, 4)) break;

        ///get message id 
        if (!readData(index + 16, (uint8_t*)&messageID, 4)) break;

        ///Check data. If not matching then go to next message in storage
    	if (messageType == messageTypeSet && dataType == dataTypeSet && (id == 0 || id == messageID)) {

            ///Message found. Exit loop.
            exit = true;

        } else {

            //Get next index
            uint32_t nextMessageIndex = 0;
            if (!readData(index + 4, (uint8_t*)&nextMessageIndex, 4)) break;
            index = nextMessageIndex;

            ///If the index is last index then that was the last message.
            if (index == getEndIndex()) break;

        }

    }

    ///Exited normally, return index.
    if (exit) return index;

    ///Break was called meaning error. 
    return 0;

}


bool DataManager_Abstract::getMessage(KraftMessage_Interface& message, uint32_t id) {

    uint32_t index = searchMessage(message.getMessageType(), message.getDataType(), id);

    //Was not found, return false.
    if (index == 0) return false;
    
    //Found out the size of data
    uint32_t size = 0;
    if (!readData(index, (uint8_t*)&size, 4)) return false;

    //Make sure sizes fit.
    if (size != message.getDataSize()) return false;

    //Get data and place in buffer.
    uint8_t buffer[size];
    if (!readData(index + 20, buffer, size)) return false;

    //Place data into message
    if (!message.setRawData(buffer, size)) return false;

    //Looks like everything worked. Signify by returning true. 
    return true;

}


bool DataManager_Abstract::setMessage(KraftMessage_Interface& message, uint32_t id) {

    uint32_t index = searchMessage(message.getMessageType(), message.getDataType(), id);

    //Was not found, return false.
    if (index == 0) return false;

    //Found out the size of data
    uint32_t size = 0;
    if (!readData(index, (uint8_t*)&size, 4)) return false;

    //Make sure sizes fit.
    if (size != message.getDataSize()) return false;

    //Get message data and place in buffer.
    uint8_t buffer[size];
    if (!message.getRawData(buffer, size)) return false;

    //Place in buffer data in storage
    if (!writeData(index + 20, buffer, size)) return false;

    //Looks like everything worked. Signify by returning true. 
    return true;

}


uint32_t DataManager_Abstract::newMessage(KraftMessage_Interface& message) {

    uint32_t index = 0;
    uint32_t nextMessageIndex = 1;
    uint32_t size = 0;
    uint32_t freeSpace = 0;

    uint32_t spaceNeeded = message.getDataSize() + 20;

    //bool exit = false;

    //Go through messages till last one.
    while (nextMessageIndex != getEndIndex() && freeSpace < spaceNeeded) {

        index = nextMessageIndex;

        //Get next index
        if (!readData(index + 4, (uint8_t*)&nextMessageIndex, 4)) break;
        //Get data size
        if (!readData(index, (uint8_t*)&size, 4)) return 0;
        size += 20;

        //Calculate free space
        freeSpace = nextMessageIndex - (index + size);
        if (index == 1) freeSpace = nextMessageIndex-9;

        //Serial.println(String("Current Index: ") + index + ", Next Index: " + nextMessageIndex + ", End Index: " + getEndIndex() + ", Size: " + freeSpace);

    }

    //Serial.println(String("Found spot at index: ") + index + ", Size: " + freeSpace);

    //determine index of new data.
    uint32_t sizeLast;
    if (!readData(index, (uint8_t*)&sizeLast, 4)) return 0;
    uint32_t newIndex = index + sizeLast + 20; //Start is byte at end of the last message.

    //If the index is 1 then this is the first message being saved to memory
    if (index == 1) newIndex = 9;

    //Serial.println(String("Placing at index: ") + newIndex);

    //Update data with parameters from message.
    uint32_t newSize = message.getDataSize();
    uint32_t newMessageType = message.getMessageType();
    uint32_t newDataType = message.getDataType();
    uint32_t newID = 1;
    uint32_t nextIndex = getEndIndex();
    if (!writeData(newIndex, (uint8_t*)&newSize, 4)) return 0;
    if (!writeData(newIndex + 4, (uint8_t*)&nextIndex, 4)) return 0;
    if (!writeData(newIndex + 8, (uint8_t*)&newMessageType, 4)) return 0;
    if (!writeData(newIndex + 12, (uint8_t*)&newDataType, 4)) return 0;
    if (!writeData(newIndex + 16, (uint8_t*)&newID, 4)) return 0;
    uint8_t buffer[newSize];
    if (!message.getRawData(buffer, newSize)) return 0;
    if (!writeData(newIndex + 20, buffer, newSize)) return 0;
    
    //Update last message with new message index. Must be done here so its only changed when everything was sucessfull.
    if (!writeData(index + 4, (uint8_t*)&newIndex, 4)) return 0;

    return newID;

}


uint32_t DataManager_Abstract::getNumberMessages(KraftMessage_Interface& message) {

    uint32_t index = 1;
    uint32_t nextMessageIndex = 0;

    uint32_t messageType = 0;
    uint32_t dataType = 0;

    uint32_t messageTypeSet = message.getMessageType();
    uint32_t dataTypeSet = message.getDataType();

    uint32_t messageCounter = 0;



    bool exit = false;

    while (!exit) {

        ///get message type and data type
        if (!readData(index + 8, (uint8_t*)&messageType, 4)) break;
        if (!readData(index + 12, (uint8_t*)&dataType, 4)) break;
        //Get the next message index
        if (readData(index + 4, (uint8_t*)&nextMessageIndex, 4)) break;

        ///Check data. If not matching then go to next message in storage
    	if (messageType == messageTypeSet && dataType == dataTypeSet) {

            messageCounter++;

        } 

        //If the next message indeex is EndIndex then that was the last message in storage and leave loop;
        if (nextMessageIndex == getEndIndex()) {

            exit = true;

        }

    }

    return exit ? messageCounter : 0;

}


void DataManager_Abstract::clear() {

    uint32_t buf = storageVersion_;
    if (!writeData(0, (uint8_t*)&buf, 1)) return;

    buf = 0;
    if (!writeData(1, (uint8_t*)&buf, 4)) return;

    buf = getEndIndex();
    if (!writeData(1 + 4, (uint8_t*)&buf, 4)) return;

}


bool DataManager_Abstract::deleteMessage(KraftMessage_Interface& message, uint32_t id) {

    uint32_t index = 1;
    uint32_t lastMessageIndex = 1;

    {

        uint32_t messageType = 0;
        uint32_t dataType = 0;
        uint32_t messageID = 0;

        uint32_t messageTypeSet = message.getMessageType();
        uint32_t dataTypeSet = message.getDataType();

        bool exit = false;

        while (!exit) {

            //get message type and data type
            if (!readData(index + 8, (uint8_t*)&messageType, 4)) break;
            if (!readData(index + 12, (uint8_t*)&dataType, 4)) break;

            //get message id 
            if (!readData(index + 16, (uint8_t*)&messageID, 4)) break;

            //Check data. If not matching then go to next message in storage
            if (messageType == messageTypeSet && dataType == dataTypeSet && (id == 0 || id == messageID)) {

                //Message found. Exit loop.
                exit = true;

            } else {

                //Get next index
                uint32_t nextMessageIndex = 0;
                if (!readData(index + 4, (uint8_t*)&nextMessageIndex, 4)) break;
                lastMessageIndex = index;
                index = nextMessageIndex;

                //If the index is last index then that was the last message.
                if (index == getEndIndex()) break;

            }

        }

        //Exited normally, return index.
        if (!exit) index = 0;

    }

    //Message not found.
    if (index == 0) return false;

    //Get index of next message.
    uint32_t nextIndex;
    if (!readData(index + 4, (uint8_t*)&nextIndex, 4)) return false;

    //Change last message next index to current message next index.
    if (!writeData(lastMessageIndex + 4, (uint8_t*)&nextIndex, 4)) return false;

    //No need to change data.
    return true;

}


uint32_t DataManager_Abstract::deleteAllMessages(KraftMessage_Interface& message) {

    uint32_t counter = 0;

    //I know this is stupid ineffiecient and has a runtime of O(n^2) but its late, works and ill fix it later(hopefully).
    while (deleteMessage(message)) counter++;

    return counter;

}


uint8_t DataManager_Abstract::operator [] (uint32_t index) {
    uint8_t buf;
    readData(index, &buf);
    return buf;
}


/*DataManager_Abstract& DataManager_Abstract::operator = (DataManager_Abstract& dataStorage) {

    for (uint32_t i = 0; i < maxDataSize_; i++) {

        this->dataPointer_[i] = dataStorage.dataPointer_[i];

    }

    this->maxDataSize_ = dataStorage.maxDataSize_;
    this->dataValid_ = dataStorage.dataValid_;
    this->currentDataSize_ = dataStorage.currentDataSize_;
    
    return *this;

}*/