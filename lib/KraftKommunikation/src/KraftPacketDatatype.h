#ifndef KRAFTPACKET_PACKETIZER_H
#define KRAFTPACKET_PACKETIZER_H



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



struct KraftPacket final {


    const uint16_t packetStart = (uint16_t)KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION;

    uint8_t transmitterID = KRAFTPACKET_PATH_IGNORE_ID;
    uint8_t receiverID = KRAFTPACKET_PATH_BROADCAST_ID;

    uint8_t packetData[255];

    uint8_t crc = 0;

    const uint8_t packetEnd = KRAFTPACKET_MARKER_ENDBYTE;

    /**
     * Packs all the packet data into the given dataBytes pointer. 
     * dataByteSize is the size of the given dataBytes buffer.
     * returns true size of packet. 0 if error.
     *
     * @param values dataBytes and dataBytes.
     * @return uint32_t.
     */
    uint32_t packDataIntoBuffer(uint8_t* dataBytes, const uint32_t &dataByteSize) {

        if (packetData == nullptr) return 0; //Make sure we have a valid data type

        if (dataByteSize < 9 + packetData->getDataSize()) return 0; //Make sure dataByte is big enough to fit all the data.

        dataBytes[0] = (uint8_t)(KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION);
        dataBytes[1] = (uint8_t)((KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)<<8);

        dataBytes[2] = transmitterID;
        dataBytes[3] = receiverID;

        uint8_t dataBuffer[packetData->getDataSize()]; //Make data buffer for data
        packetData->getRawData(dataBuffer, sizeof(dataBuffer)); //Transfer data into databuffer

        uint32_t dataID = packetData->getDataTypeID();
        uint32_t dataSize = packetData->getDataSize();

        dataBytes[4] = dataID;
        dataBytes[5] = dataSize;
        
        for (uint32_t i = 0; i < sizeof(dataBuffer); i++) dataBytes[6+i] = dataBuffer[i]; //Move data from buffer into output

        dataBytes[dataSize + 6] = crc = calculateCRC();

        dataBytes[dataSize + 7] = KRAFTPACKET_MARKER_ENDBYTE;

        return dataSize + 7 + 1;

    }


    /**
     * Unpacks the raw dataBytes packet into the class.
     * dataByteSize is the size of the given dataBytes buffer.
     * returns false if error.
     *
     * @param values dataBytes and dataBytes.
     * @return bool.
     */
    bool unpackDataFromBuffer(const uint8_t* dataBytes, const uint32_t &dataByteSize) {

        /*if (packetData == nullptr) return 0; //Make sure we have a valid data type

        if (dataByteSize < 9 + packetData->getDataSize()) return 0; //Make sure dataByte is big enough to fit all the data.

        dataBytes[0] = (uint8_t)(KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION);
        dataBytes[1] = (uint8_t)((KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)<<8);

        dataBytes[2] = transmitterID;
        dataBytes[3] = receiverID;

        uint8_t dataBuffer[packetData->getDataSize()]; //Make data buffer for data
        packetData->getRawData(dataBuffer, sizeof(dataBuffer)); //Transfer data into databuffer

        uint32_t dataID = packetData->getDataTypeID();
        uint32_t dataSize = packetData->getDataSize();

        dataBytes[4] = dataID;
        dataBytes[5] = dataSize;
        
        for (uint32_t i = 0; i < sizeof(dataBuffer); i++) dataBytes[6+i] = dataBuffer[i]; //Move data from buffer into output

        dataBytes[dataSize + 6] = crc = calculateCRC();

        dataBytes[dataSize + 7] = KRAFTPACKET_MARKER_ENDBYTE;

        return dataSize + 7 + 1;*/

    }


    /**
     * Calculates the packets crc byte
     *
     * @param values none.
     * @return uint8_t.
     */
    uint8_t calculateCRC() {

       uint8_t crc = 0;

       crc += (uint8_t)(KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION);
       crc += (uint8_t)((KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)<<8);
       crc += transmitterID;
       crc += receiverID;
       crc += dataID;
       crc += dataSize;

       if (packetData != nullptr) {

            uint8_t dataBuffer[packetData->getDataSize()]; //Make data buffer for data
            packetData->getRawData(dataBuffer, sizeof(dataBuffer)); //Transfer data into databuffer

            uint32_t dataID = packetData->getDataTypeID();
            uint32_t dataSize = packetData->getDataSize();
            
            for (uint32_t i = 0; i < sizeof(dataBuffer); i++) crc += dataBuffer[i];

       }

       return crc;

    }

};




#endif 
