#ifndef KRAFTPACKET_DATA_TYPE_H
#define KRAFTPACKET_DATA_TYPE_H



#include "stdint.h"



#define KRAFTPACKET_PACKETTYPE_HEARTBEAT_ID 0



class KraftDataType {
public:

    virtual uint32_t getDataTypeID() = 0;

    virtual uint32_t getDataSize() = 0;

    virtual bool getRawData(uint8_t* dataBytes, const uint32_t &dataByteSize) = 0;

    virtual bool setRawData(uint8_t* dataBytes, const uint32_t &dataByteSize) = 0;

protected:

};


class HeartbeatPacket final: public KraftDataType {

    uint32_t getDataTypeID() {return KRAFTPACKET_PACKETTYPE_HEARTBEAT_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(uint8_t* dataBytes, const uint32_t &dataByteSize) {
        return false;
    }

    bool setRawData(uint8_t* dataBytes, const uint32_t &dataByteSize) {
        return false;
    }

};



#endif 
