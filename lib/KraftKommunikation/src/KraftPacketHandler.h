#ifndef KRAFTPACKET_HANDLER_H
#define KRAFTPACKET_HANDLER_H



#include "stdint.h"

#include "KraftPacketDataType.h"



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

//Should stay below 255 due to most packet markers being uint8_t datatypes
#define KRAFTPACKET_SIZE_LIMIT_BYTES 200



//Is the ID of a devide like Vehicle or basestation. Broadcast means the data is meant for all devices to receive. Also used for Transmitting device ID.
enum KRAFTPACKET_PATH_ID {
    KRAFTPACKET_PATH_IGNORE_ID,             //Ignore Messages with this ID
    KRAFTPACKET_PATH_VEHICLE_ID,            //Vehicle ID
    KRAFTPACKET_PATH_CONTROLLER_ID,         //Controllers ID. Controller is for manual control
    KRAFTPACKET_PATH_BASESTATION_ID,        //Basestation ID. 
    KRAFTPACKET_PATH_BROADCAST_ID           //General packet that everything can receive
};


class KraftPacket_Small final {
public:

    /**
     * Takes the Data from the given parameter and readys packet.
     * 
     * Returns data type ID. Will be 0 If failed.
     *
     * @param values packetData.
     * @return uint32_t.
     */
    uint32_t setPacketData(KraftDataType &packetData) {

        if (packetData.getDataSize() > KRAFTPACKET_SIZE_LIMIT_BYTES) return 0;

        uint8_t dataBuffer[packetData.getDataSize()]; //Make data buffer for data
        packetData.getRawData(_packetBytes, KRAFTPACKET_SIZE_LIMIT_BYTES); //Transfer data into databuffer

        _dataID = packetData.getDataTypeID();
        _dataSize = packetData.getDataSize();

        return _dataID;

    }


    /**
     * Puts unpacked Data into the given KraftDataType pointer.
     * 
     * Returns false if failed
     *
     * @param values packetData.
     * @return bool.
     */
    bool getPacketData(KraftDataType* packetData) {

        if (packetData == nullptr) return false;

        if (packetData->getDataSize() != _dataSize || packetData->getDataTypeID() != _dataID) return false;

        return packetData->setRawData(_packetBytes, KRAFTPACKET_SIZE_LIMIT_BYTES); //Transfer data into packet

    }


    /**
     * Packs all the packet data into the given dataBytes pointer. 
     * dataByteSize is the size of the given dataBytes buffer.
     * returns true size of packet. 0 if error.
     *
     * @param values dataBytes and dataBytes.
     * @return uint32_t.
     */
    uint32_t packDataIntoBuffer(uint8_t* dataBytes, const uint32_t &dataByteSize) {

        if (_dataID == 0) return 0; //Make sure we have a valid data type

        if (dataByteSize < 9 + _dataSize) return 0; //Make sure dataByte is big enough to fit all the data.

        dataBytes[0] = (uint8_t)(KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION);
        dataBytes[1] = (uint8_t)((KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)<<8);

        dataBytes[2] = _transmitterID;
        dataBytes[3] = _receiverID;

        dataBytes[4] = _dataID;
        dataBytes[5] = _dataSize;
        
        for (uint32_t i = 0; i < sizeof(_packetBytes); i++) dataBytes[6+i] = _packetBytes[i]; //Move data from buffer into output

        dataBytes[_dataSize + 6] = _crc = calculateCRC();

        dataBytes[_dataSize + 7] = KRAFTPACKET_MARKER_ENDBYTE;

        return _dataSize + 7 + 1;

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

        if (dataBytes[0] != (uint8_t)(KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)) return false;
        if (dataBytes[1] != (uint8_t)((KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION)<<8)) return false;

        _transmitterID = dataBytes[2];
        _receiverID = dataBytes[3];

        _dataID = dataBytes[4];
        _dataSize = dataBytes[5];

        if (_dataSize > dataByteSize) return false;
        
        for (uint32_t i = 0; i < _dataSize; i++) _packetBytes[i] = dataBytes[6+i]; //Move data from buffer into output

        if (dataBytes[_dataSize + 7] != KRAFTPACKET_MARKER_ENDBYTE) return false;

        _crc = calculateCRC();

        if (dataBytes[_dataSize + 6] != _crc) return false;

        return true;

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
        crc += _transmitterID;
        crc += _receiverID;
        crc += _dataID;
        crc += _dataSize;
            
        for (uint32_t i = 0; i < _dataSize; i++) crc += _packetBytes[i];

       return crc;

    }



private:

    const uint16_t _packetStart = (uint16_t)KRAFTPACKET_MARKER_STARTINTEGER + KRAFTPACKET_VERSION;

    uint8_t _transmitterID = KRAFTPACKET_PATH_IGNORE_ID;
    uint8_t _receiverID = KRAFTPACKET_PATH_BROADCAST_ID;

    uint32_t _dataID = 0;
    uint32_t _dataSize = 0;

    uint8_t _packetBytes[KRAFTPACKET_SIZE_LIMIT_BYTES];

    uint8_t _crc = 0;

    const uint8_t _packetEnd = KRAFTPACKET_MARKER_ENDBYTE;

};




#endif 
