#include "kraft_kommunication.h"



bool KraftKommunication::decodeMessageFromBuffer(ReceivedPayloadData* payloadData, bool* ackRequested, uint8_t* buffer, uint32_t bufferSize) {

    if (buffer[0] != (uint8_t)(c_kraftPacketStartMarker + c_kraftPacketVersion)) return false;
    if (buffer[1] != (uint8_t)((c_kraftPacketStartMarker + c_kraftPacketVersion)<<8)) return false;


    payloadData->messageData.transmitterID = static_cast<kraftPacketNodeID_t>(buffer[2]);
    payloadData->messageData.receiverID = static_cast<kraftPacketNodeID_t>(buffer[3]);

    if (payloadData->messageData.transmitterID == kraftPacketNodeID_t::kraftPacketNodeID_basestation) return false; //That would be pretty wierd if the sender was a broadcast id

    payloadData->messageData.payloadID = buffer[4];

    *ackRequested = buffer[5];
    
    payloadData->messageData.messageCounter = buffer[6];

    uint32_t payloadSize = buffer[7];

    if (payloadSize > bufferSize) return false;
    
    for (uint32_t i = 0; i < payloadSize; i++) payloadData->dataBuffer[i] = buffer[8+i]; //Move data from buffer into output

    uint8_t crc = buffer[8+payloadSize];

    if (buffer[9+payloadSize] != c_kraftPacketEndMarker) return false;

    //if (crc != calculateCRC(buffer, payloadSize + 7)) return false; //Commented for testing, Should be readded and tested later!

    return true;

}



uint32_t KraftKommunication::encodeMessageToBuffer(KraftMessage_Interface* messagePointer, const uint8_t &receiveNode, const bool &requestAck, uint8_t* buffer, const uint32_t &bufferSize) {

    if (bufferSize < 9 + messagePointer->getDataSize()) return 0; //Make sure dataByte is big enough to fit all the data.

    buffer[0] = (uint8_t)(c_kraftPacketStartMarker + c_kraftPacketVersion);
    buffer[1] = (uint8_t)((c_kraftPacketStartMarker + c_kraftPacketVersion)<<8);

    buffer[2] = selfID_;
    buffer[3] = receiveNode;

    buffer[4] = messagePointer->getDataTypeID();

    buffer[5] = requestAck;

    buffer[6] = sentPacketsCounter_[receiveNode];

    uint32_t messageSize = messagePointer->getDataSize();
    buffer[7] = messageSize;

    uint8_t messageData[messageSize];

    messagePointer->getRawData(messageData, messageSize);
    
    for (uint32_t i = 0; i < sizeof(bufferSize) && messageSize; i++) buffer[8+i] = messageData[i]; //Move data from buffer into output

    buffer[messageSize + 8] = calculateCRC(buffer, messageSize + 7);

    buffer[messageSize + 9] = c_kraftPacketEndMarker;

    return messageSize + 9 + 1;

}



uint8_t KraftKommunication::calculateCRC(uint8_t* buffer, const uint32_t &stopByte) {

    uint8_t crc = 0;

    for (uint32_t i = 0; i < stopByte; i++) crc += buffer[i];

    return crc;

}
