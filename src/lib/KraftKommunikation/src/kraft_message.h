#ifndef KRAFT_MESSAGE_H
#define KRAFT_MESSAGE_H



#include "stdint.h"

#include "string.h"



//Indicates the type of packet. Use eKraftMessageType_StandardEnd_ID and up for custom Packets
enum eKraftMessageType_t : uint8_t {
    //Simply signals that connection is still there. Should be send at 1Hz rate.
    eKraftMessageType_Heartbeat_ID,
    //To be returned to signal succesfull receive when requiested.
    eKraftMessageType_Ack_ID,
    //Generic packet with parameters for updating RadioSettings.
    eKraftMessageType_RadioSettings_ID,
    //Used for sending strings.
    eKraftMessageType_String_ID,
    //Use numbers higher than this for custom IDs
    eKraftMessageType_StandardEnd_ID
};



class KraftMessage_Interface {
public:

    virtual uint32_t getDataTypeID() = 0;

    virtual uint32_t getDataSize() = 0;

    virtual bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) = 0;

    virtual bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) = 0;

};



class KraftMessageHeartbeat final: public KraftMessage_Interface {
public:

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_Heartbeat_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return false;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return false;
    }

};



class KraftMessageACK final: public KraftMessage_Interface {
public:

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_Ack_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return false;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return false;
    }

};



/**
 * Not recommended. Please create a class that inherets from KraftMessagePacket_Abstract
 */
/*
template<typename T>
class KraftMessageTemplate final: public KraftMessage_Interface {
public:

    KraftMessageTemplatePacket(const T &data, eKraftMessageType_t messageID) {
        data_ = data;
        messageID_ = messageID_;
    }

    uint32_t getDataTypeID() {return messageID_;}

    uint32_t getDataSize() {return sizeof(T);};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < (sizeof(data_) + startByte)) return false;
        
        for (uint32_t i = 0; i < (dataByteSize-startByte) && i < sizeof(T); i++) ((uint8_t*)dataBytes)[i+startByte] = ((uint8_t*)data_)[i];

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < (sizeof(data_)+startByte)) return false;
        
        for (uint32_t i = 0; i < (dataByteSize-startByte) && i < sizeof(T); i++) ((uint8_t*)data_)[i] = ((uint8_t*)dataByteSize)[i+startByte];

        return true;

    }

private:

    T data_;
    uint32_t messageID_ = 0;

};*/



class KraftMessageStringPacket final: public KraftMessage_Interface {
public:

    KraftMessageStringPacket() {}

    KraftMessageStringPacket(const char string[]) {
        setString(string);
    }

    ~KraftMessageStringPacket() {
        delete[] stringPointer;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_String_ID;}

    uint32_t getDataSize() {return sizeStringPointer;};

    uint32_t getStringLength() {return sizeStringPointer;}

    bool getString(char string[], const uint32_t &sizeStringDest) {

        if (sizeStringDest < sizeStringPointer || stringPointer == nullptr) return false;

        memcpy(string, stringPointer, sizeStringPointer);

        return true;

    }

    void setString(const char string[]) {
        
        if (stringPointer != nullptr) delete[] stringPointer;

        sizeStringPointer = strlen(string)+1;
        stringPointer = new char[sizeStringPointer];

        memcpy(stringPointer, string, sizeStringPointer);

    }

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        return getString((char*)dataBytes+startByte, dataByteSize);

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        setString((char*)(dataBytes+startByte));

        return true;

    }


private:

    char* stringPointer = nullptr;
    uint32_t sizeStringPointer = 0;

};



#endif 
