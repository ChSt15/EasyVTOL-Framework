#include "KraftKontrol/utils/KraftKommunikation/kraft_kommunication.h"



bool KraftKommunication::decodeMessageFromBuffer(ReceivedPayloadData* payloadData, bool* ackRequested, uint8_t* buffer, uint32_t bufferSize) {

    if (buffer[0] != (uint8_t)(c_kraftPacketStartMarker + c_kraftPacketVersion)) return false;
    if (buffer[1] != (uint8_t)((c_kraftPacketStartMarker + c_kraftPacketVersion)<<8)) return false;


    payloadData->messageData.transmitterID = static_cast<eKraftPacketNodeID_t>(buffer[2]);
    payloadData->messageData.receiverID = static_cast<eKraftPacketNodeID_t>(buffer[3]);

    if (payloadData->messageData.transmitterID == eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast) return false; //That would be pretty wierd if the sender was a broadcast id

    payloadData->messageData.payloadID = buffer[4];

    *ackRequested = buffer[5];
    
    payloadData->messageData.messageCounter = buffer[6];

    payloadData->messageData.payloadSize = buffer[7];

    if (payloadData->messageData.payloadSize > bufferSize) return false;
    

    for (uint32_t i = 0; i < payloadData->messageData.payloadSize; i++) payloadData->dataBuffer[i] = buffer[8+i]; //Move data from buffer into output

    //uint8_t crc = buffer[8+payloadData->messageData.payloadSize];

    if (buffer[9+payloadData->messageData.payloadSize] != c_kraftPacketEndMarker) return false;

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
    
    for (uint32_t i = 0; i < bufferSize && messageSize; i++) buffer[8+i] = messageData[i]; //Move data from buffer into output

    buffer[messageSize + 8] = calculateCRC(buffer, messageSize + 7);

    buffer[messageSize + 9] = c_kraftPacketEndMarker;

    return messageSize + 9 + 1;

}



uint8_t KraftKommunication::calculateCRC(uint8_t* buffer, const uint32_t &stopByte) {

    uint8_t crc = 0;

    for (uint32_t i = 0; i < stopByte; i++) crc += buffer[i];

    return crc;

}



bool KraftKommunication::sendMessage(KraftMessage_Interface* kraftMessage, const eKraftPacketNodeID_t &receiveNodeID, const bool &requiresAck) {

    SendPacketData packetData;

    packetData.bufferSize = encodeMessageToBuffer(kraftMessage, receiveNodeID, false, packetData.dataBuffer, sizeof(packetData.dataBuffer));

    if (packetData.bufferSize == 0) return false; //encode failure if buffer size is 0. Return false to indicate failure.

    packetData.power_dB = c_sendPowerDefault;
    packetData.receivingNodeID = receiveNodeID;
    packetData.waitforAck = requiresAck;

    if (packetData.waitforAck) {

        packetData.sendAttempts = c_sendAttempts;
        packetData.sendInterval = c_sendTimeout/c_sendAttempts;

    }

    if (requiresAck) sendPacketsACK_.placeFront(packetData);
    else sendPackets_.placeFront(packetData);

    return true;

}



bool KraftKommunication::getMessage(KraftMessage_Interface* kraftMessage, const bool &peek) {

    ReceivedPayloadData payloadData;
    
    if (!peek) receivedPackets_.takeBack(&payloadData);
    else receivedPackets_.peekBack(&payloadData);

    return kraftMessage->setRawData(payloadData.dataBuffer, payloadData.messageData.payloadSize);

}



void KraftKommunication::loop() {


    if (!dataLink_->busy()) {

        if (sendPackets_.available()) {

            SendPacketData packet;
            sendPackets_.takeBack(&packet);
            dataLink_->sendBuffer(packet.dataBuffer, packet.bufferSize);
            
        } else if (sendPacketsACK_.available()) {

            SendPacketData* packet = &sendPacketsACK_[0];

            if (nodeData_[packet->receivingNodeID].waitingOnPacket == nullptr) { 

                dataLink_->sendBuffer(packet->dataBuffer, packet->bufferSize);

                if (packet->sendAttempts > 0) packet->sendAttempts--;
                packet->sendTimestamp = micros();

                nodeData_[packet->receivingNodeID].waitingOnPacket = packet;

            } else {

                if (packet->sendTimestamp + packet->sendInterval < micros()) {
                    
                    if (packet->sendAttempts == 0) {

                        sendPacketsACK_.removeBack();
                        nodeData_[packet->receivingNodeID].waitingOnPacket = nullptr;

                    } else {

                        packet->sendAttempts--;
                        packet->sendTimestamp = micros();

                        dataLink_->sendBuffer(packet->dataBuffer, packet->bufferSize);

                    }
                    
                }

            }

        }

    }



    if (dataLink_->available()) {

        //Serial.println("Kraftkomm sees packet is available!");

        uint8_t packet[dataLink_->available()];

        uint32_t bytes = dataLink_->receiveBuffer(packet, sizeof(packet));

        //Serial.println("Kraft komm sees packet is " + String(bytes) + " bytes long!");

        if (bytes > 0) { //Only continue if receive worked.

            ReceivedPayloadData message;
            bool ackRequested;

            if (decodeMessageFromBuffer(&message, &ackRequested, packet, sizeof(packet))) {

                if (ackRequested) {
                    
                    KraftMessageACK ack;
                    sendMessage(&ack, message.messageData.transmitterID);

                }

                if (message.messageData.receiverID == selfID_ || message.messageData.receiverID == eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast) { //Make sure packet is for us.

                    nodeData_[message.messageData.transmitterID].lastPacketTimestamp = millis();
                    nodeData_[message.messageData.transmitterID].online = true;

                    switch (message.messageData.payloadID) {
                    case eKraftMessageType_t::eKraftMessageType_Ack_ID:
                        
                        sendPacketsACK_.removeBack();
                        nodeData_[message.messageData.transmitterID].waitingOnPacket = nullptr; 

                        //Serial.println("Packet was an ACK");

                        break;

                    case eKraftMessageType_t::eKraftMessageType_Heartbeat_ID:

                        // Serial.println("Packet was a heartbeat");
                        receivedPackets_.placeFront(message);

                        break;

                    case eKraftMessageType_t::eKraftMessageType_RadioSettings_ID:

                        //Serial.println("Packet was for radiosettings");
                        receivedPackets_.placeFront(message);

                        break;

                    default:

                        //Serial.println("Packet was of an unkown type: " + String(message.messageData.payloadID) + ". Placing in buffer.");
                        receivedPackets_.placeFront(message);
                        
                        break;

                    } 

                } else {

                    //Serial.println("Packet wasnt for me");

                }

            } else {

                //Serial.println("DECODE FAILED!");

            }

        }

    }


    //Check for timeout on all nodes.
    for (uint8_t i = 0; i < 0; i++) {

        if (millis() - nodeData_[i].lastPacketTimestamp >= 500) {
            
            nodeData_[i].online = false;

        }

    }


    

}
