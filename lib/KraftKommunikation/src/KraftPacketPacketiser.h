#ifndef KRAFTPACKET_DATA_TYPE_H
#define KRAFTPACKET_DATA_TYPE_H



#include "stdint.h"



/**
 * Packet transfer algorithm:
 * ...
 * 
 * Packet Structure:
 * n is the size if the packets data.
 * - First 2 bytes are the Kraftpacket startbyte markers. These must be first checked to make sure the Packet is actually a kraftpacket and correct version
 * - 3rd byte is the transmitterID
 * - 4th byte is the receiverID. A packet not adressed to a receiver should be ignored.
 * - 5th byte is the packet incrementer. This is incremented for every new byte addressed to a specific reciever. Used for counting lost packets
 * - 6 to (5+n)th byte is the raw data
 * - (6+n)th byte is the crc byte to check if data is valid
 * - (7+n) is the endbyte marker
 */



#define KRAFTPACKET_MARKER_STARTINTEGER 0xAABB
#define KRAFTPACKET_MARKER_ENDBYTE 0xAB

#define KRAFTPACKET_VERSION 0

#define KRAFTPACKET_PATH_IGNORE_ID 0 //Ignore Messages with this ID
#define KRAFTPACKET_PATH_VEHICLE_ID 1 //Vehicles ID
#define KRAFTPACKET_PATH_CONTROLLER_ID 2 //Controllers ID. Controller is for manual control
#define KRAFTPACKET_PATH_BASESTATION_ID 200 //Basestation ID. 
#define KRAFTPACKET_PATH_BROADCAST_ID 255 //General packet that everything can receive

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
