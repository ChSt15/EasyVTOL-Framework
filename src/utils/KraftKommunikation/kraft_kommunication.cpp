#include "KraftKontrol/modules/communication_modules/kraft_kommunication.h"



bool KraftKommunication::decodeMessageFromBuffer(ReceivedPayloadData& payloadData, bool& ackRequested, uint8_t* buffer, uint32_t bufferSize) {

    if (buffer[0] != (uint8_t)(c_kraftPacketStartMarker + c_kraftPacketVersion)) return false;
    if (buffer[1] != (uint8_t)((c_kraftPacketStartMarker + c_kraftPacketVersion)<<8)) return false;


    payloadData.messageData.transmitterID = static_cast<eKraftMessageNodeID_t>(buffer[2]);
    payloadData.messageData.receiverID = static_cast<eKraftMessageNodeID_t>(buffer[3]);

    if (payloadData.messageData.transmitterID == eKraftMessageNodeID_t::eKraftMessageNodeID_broadcast) return false; //That would be pretty wierd if the sender was a broadcast id

    payloadData.messageData.messageTypeID = buffer[4];
    payloadData.messageData.dataTypeID = buffer[5];

    ackRequested = buffer[6];
    
    payloadData.messageData.messageCounter = buffer[7];

    payloadData.messageData.payloadSize = buffer[8];

    if (payloadData.messageData.payloadSize > bufferSize) return false;
    

    for (uint32_t i = 0; i < payloadData.messageData.payloadSize; i++) payloadData.dataBuffer[i] = buffer[9+i]; //Move data from buffer into output

    uint8_t crc = buffer[9+payloadData.messageData.payloadSize];

    if (buffer[10+payloadData.messageData.payloadSize] != c_kraftPacketEndMarker) return false;

    if (crc != calculateCRC(buffer, payloadData.messageData.payloadSize + 8)) return false; //Commented for testing, Should be readded and tested later!

    return true;

}



uint32_t KraftKommunication::encodeMessageToBuffer(const KraftMessage_Interface& message, uint8_t receiveNode, bool requestAck, uint8_t* buffer, uint32_t bufferSize) {

    if (bufferSize < 9 + message.getDataSize()) return 0; //Make sure dataByte is big enough to fit all the data.

    buffer[0] = (uint8_t)(c_kraftPacketStartMarker + c_kraftPacketVersion);
    buffer[1] = (uint8_t)((c_kraftPacketStartMarker + c_kraftPacketVersion)<<8);

    buffer[2] = selfID_;
    buffer[3] = receiveNode;

    buffer[4] = message.getMessageType();
    buffer[5] = message.getDataType();

    buffer[6] = requestAck;

    buffer[7] = sentPacketsCounter_[receiveNode];

    uint32_t messageSize = message.getDataSize();
    buffer[8] = messageSize;

    uint8_t messageData[messageSize];

    message.getRawData(messageData, messageSize);
    
    for (uint32_t i = 0; i < bufferSize && messageSize; i++) buffer[9+i] = messageData[i]; //Move data from buffer into output

    buffer[messageSize + 9] = calculateCRC(buffer, messageSize + 8);

    buffer[messageSize + 10] = c_kraftPacketEndMarker;

    return messageSize + 10 + 1;

}



uint8_t KraftKommunication::calculateCRC(uint8_t* buffer, uint32_t stopByte) {

    uint8_t crc = 0;

    for (uint32_t i = 0; i < stopByte; i++) crc += buffer[i];

    return crc;

}



bool KraftKommunication::sendMessage(const KraftMessage_Interface& kraftMessage, eKraftMessageNodeID_t receiveNodeID, bool requiresAck) {

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



void KraftKommunication::kraftMessageSendCallback(const MessageLinkData& item) {

    sendMessage(item.message, item.receiverID, item.requiresAck);

}



void KraftKommunication::kraftMessageBroadcastCallback(const KraftMessage_Interface& item) {

    sendMessage(item, eKraftMessageNodeID_t::eKraftMessageNodeID_broadcast);

}



void KraftKommunication::thread() {

    //Check node status and if a packet needs to be sent.
    for (uint32_t i = 0; i < c_maxNumberNodes; i++) {
        if (nodeData_[i].online && NOW() - nodeData_[i].lastPacketTimestamp > (int64_t)SECONDS*3/c_heartbeatRate) {
            nodeData_[i].online = false;
            //KraftMessageEmulator message = DataMessageNodeStatus(false, i);
            globalMessages().publish(DataMessageNodeStatus(false, i));
        }
    }
    if (heartbeatTimer_.isTimeToRun()) {

        KraftMessageHeartbeat message;
        sendMessage(message, eKraftMessageNodeID_t::eKraftMessageNodeID_broadcast);

    }


    //Send packets waiting to be sent
    while (!dataLink_.busy() && (sendPackets_.available() > 0 || sendPacketsACK_.available() > 0)) {

        if (sendPackets_.available()) {

            SendPacketData packet;
            sendPackets_.takeBack(packet);
            DataMessageBuffer message;
            message.setBuffer(packet.dataBuffer, packet.bufferSize);
            heartbeatTimer_.syncClock();
            dataLinkSendSubr_.publish(message);
            
        } else if (sendPacketsACK_.available()) {

            SendPacketData* packet = &sendPacketsACK_[0];

            if (nodeData_[packet->receivingNodeID].waitingOnPacket == nullptr) { 

                DataMessageBuffer message;
                message.setBuffer(packet->dataBuffer, packet->bufferSize);
                heartbeatTimer_.syncClock();
                dataLinkSendSubr_.publish(message);

                if (packet->sendAttempts > 0) packet->sendAttempts--;
                packet->sendTimestamp = NOW();

                nodeData_[packet->receivingNodeID].waitingOnPacket = packet;

            } else {

                if (packet->sendTimestamp + packet->sendInterval < NOW()) {
                    
                    if (packet->sendAttempts == 0) {

                        sendPacketsACK_.removeBack();
                        nodeData_[packet->receivingNodeID].waitingOnPacket = nullptr;

                    } else {

                        packet->sendAttempts--;
                        packet->sendTimestamp = NOW();

                        DataMessageBuffer message;
                        message.setBuffer(packet->dataBuffer, packet->bufferSize);
                        heartbeatTimer_.syncClock();
                        dataLinkSendSubr_.publish(message);

                    }
                    
                }

            }

        }

    }


    //Receive any packets received by data link
    if (receivedPacketSub_.available() > 0) {

        //Serial.println("Kraftkomm sees packet is available!");

        DataMessageBuffer message;
        receivedPacketSub_.takeBack(message);

        uint32_t bytes = message.getBufferSize();
        uint8_t packet[bytes];

        for (uint32_t i = 0; i < bytes; i++) packet[i] = message.getBuffer()[i];

        //Serial.println("Kraft komm sees packet is " + String(bytes) + " bytes long!");

        if (bytes > 0) { //Only continue if there is data

            ReceivedPayloadData message;
            bool ackRequested;

            if (decodeMessageFromBuffer(message, ackRequested, packet, bytes)) {

                if (ackRequested) {
                    
                    KraftMessageACK messageAck;
                    sendMessage(messageAck, message.messageData.transmitterID);

                }

                if (message.messageData.receiverID == selfID_ || message.messageData.receiverID == eKraftMessageNodeID_t::eKraftMessageNodeID_broadcast) { //Make sure packet is for us.

                    nodeData_[message.messageData.transmitterID].lastPacketTimestamp = NOW();

                    if (!nodeData_[message.messageData.transmitterID].online) {

                        nodeData_[message.messageData.transmitterID].online = true;
                        globalMessages().publish(DataMessageNodeStatus(true, message.messageData.transmitterID));

                    }

                    KraftMessageEmulator kraftMessage(message.dataBuffer, message.messageData.payloadSize, message.messageData.messageTypeID, message.messageData.dataTypeID);

                    if (message.messageData.messageTypeID == eKraftMessageType_t::eKraftMessageType_Datalink_ID) {

                        switch (message.messageData.dataTypeID) {
                        case eDataLinkDataType_t::eDataLinkDataType_ACK:
                            sendPacketsACK_.removeBack();
                            nodeData_[message.messageData.transmitterID].waitingOnPacket = nullptr; 
                            break;

                        case eDataLinkDataType_t::eDataLinkDataType_Heartbeat:
                            //NEEDS IMPLEMENTATION. Should reset timer for checking if a device is connected.
                            break;
                        
                        default:
                            break;

                        }

                    }

                    //Publish new receive message to topic.
                    receivedMessagesTopic_.publish(kraftMessage);

                } else {

                    //Serial.println("Packet wasnt for me");

                }

            } else {

                //Serial.println("DECODE FAILED!");

            }

        }

    }
    

}



