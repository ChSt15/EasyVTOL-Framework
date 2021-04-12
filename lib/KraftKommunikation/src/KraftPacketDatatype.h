#ifndef KRAFTPACKET_DATA_TYPE_H
#define KRAFTPACKET_DATA_TYPE_H



#include "stdint.h"



#define KRAFTPACKET_DATA_FREE_ID 50



enum KRAFTPACKET_DATA_STANDARD_ID {
    KRAFTPACKET_DATA_FAILURE_ID,
    KRAFTPACKET_DATA_HEARTBEAT_ID,
    KRAFTPACKET_DATA_STRING_ID
};


class KraftDataType {
public:

    virtual uint32_t getDataTypeID() = 0;

    virtual uint32_t getDataSize() = 0;

    virtual bool getRawData(void* dataBytes, const uint32_t &dataByteSize) = 0;

    virtual bool setRawData(const void* dataBytes, const uint32_t &dataByteSize) = 0;

protected:

};


class KraftDataHeartbeatPacket final: public KraftDataType {
public:

    uint32_t getDataTypeID() {return KRAFTPACKET_DATA_STANDARD_ID::KRAFTPACKET_DATA_HEARTBEAT_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize) {
        return false;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize) {
        return false;
    }

};


class KraftDataStringPacket final: public KraftDataType {
public:

    ~KraftDataStringPacket() {
        delete stringPointer;
    }

    uint32_t getDataTypeID() {return KRAFTPACKET_DATA_STANDARD_ID::KRAFTPACKET_DATA_STRING_ID;}

    uint32_t getDataSize() {return 0;};

    bool getString(char* string, uint32_t sizeString) {

        if (sizeString < sizeStringPointer || stringPointer == nullptr) return false;

        for (uint32_t i = 0; i < sizeString && i < sizeStringPointer; i++) string[i] = stringPointer[i];

        return true;

    }

    void setString(char* string, uint32_t sizeString) {
        
        delete stringPointer;

        stringPointer = new uint8_t[sizeString];
        sizeStringPointer = sizeString;

        for (uint32_t i = 0; i < sizeString; i++) stringPointer[i] = string[i];

    }

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize) {

        return getString((char*)dataBytes, dataByteSize);

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize) {

        setString((char*)dataBytes, dataByteSize);

        return true;

    }


private:

    uint8_t* stringPointer = nullptr;
    uint32_t sizeStringPointer = 0;

};



#endif 
