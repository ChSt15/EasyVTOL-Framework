#ifndef KRAFT_KOMMUNICATION_H
#define KRAFT_KOMMUNICATION_H



#include "Arduino.h"

#include "stdint.h"

#include "circular_buffer.h"

#include "kraft_link.h"
#include "kraft_message.h"


/**
 * Packet Structure:
 * n is the size if the packets data.
 * - First byte 0 and 1 are the Kraftpacket startbyte markers. These must be first checked to make sure the Packet is actually a kraftpacket and correct version
 * - byte 2 is the transmitterID
 * - byte 3 is the receiverID. A packet not adressed to a receiver should be ignored.
 * - byte 4 indicates what type of kraftmessage.
 * - byte 5 is a flag for if an ACK packet must be returned.
 * - byte 6 is the packet incrementer. This is incremented for every new byte addressed to a specific reciever. Used for counting lost packets.
 * - byte 7 is the size of the packets payload (n ist this number).
 * - byte 8 to (7+n)th byte is the raw data
 * - (8+n)th byte is the crc byte to check if data is valid
 * - (9+n) is the endbyte marker
 */



namespace {

    const uint16_t c_kraftPacketStartMarker = 0xBD5E;
    const uint8_t c_kraftPacketEndMarker = 0xDE;

    const uint16_t c_kraftPacketVersion = 0; //Should be changed when something in the protocoll was changed.

    //Number of times a packet will be sent if an ack is required.
    const uint8_t c_sendAttempts = 10;
    //Amount of microseconds to wait for an ack before resending after the link says the packet was sent, not when given the packet.
    const uint32_t c_sendTimeout = 500; 
    //Default power to send packets at.
    const uint8_t c_sendPowerDefault = 10;

}  




//Is the ID of a devide like Vehicle or basestation. Broadcast means the data is meant for all devices to receive. Also used for Transmitting device ID.
enum eKraftPacketNodeID_t : uint8_t {
    eKraftPacketNodeID_vehicle = 1,                //Vehicle ID
    eKraftPacketNodeID_controller = 2,             //Controllers ID. Controller is for manual control
    eKraftPacketNodeID_basestation = 3,            //Basestation ID. 
    eKraftPacketNodeID_broadcast = 255             //General packet that everything can receive. Basically a broadcast.
};



struct MessageData {

    eKraftPacketNodeID_t transmitterID;
    eKraftPacketNodeID_t receiverID;
    uint8_t payloadID = 0;
    uint8_t payloadSize = 0;
    uint8_t messageCounter = 0;

    uint8_t signalRSSI = 0;
    uint8_t signalSNR = 0;

};



class KraftKommunication {
public:

    /**
     * Creates a KraftKommunication object to handle a radio object and its packets
     * 
     * @param dataLink is a radio class that inherets from KraftLink_Interface that will be used to send and receive packets 
     */
    KraftKommunication(KraftLink_Interface* dataLink, eKraftPacketNodeID_t selfID) {
        dataLink_ = dataLink;
        selfID_ = selfID;
    }

    /**
     * Sends the given message.
     * 
     * @param kraftMessage is a KraftMessage_Interface class containing the data to be sent.
     * @returns false if queue is full. loop() needs to be ran to give link time to send data and make room.
     */
    bool sendMessage(KraftMessage_Interface* kraftMessage, const eKraftPacketNodeID_t &receiveNodeID, const bool &requiresAck = false);

    /**
     * Checks if a message is available.
     * 
     * @returns Number of messages available
     */
    uint16_t messageAvailable() {return receivedPackets_.available();}

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @returns MessageData struct.
     */
    MessageData getMessageInformation() {
        ReceivedPayloadData buf = receivedPackets_.pop_back();
        receivedPackets_.push_back(buf);
        return buf.messageData;
    };

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @param kraftMessage is a pointer to a class that inherets from KraftMessage_Interface.
     * @param peek is a bool. If set to true then the message will not be removed from queue. Default value if false.
     * @returns true if message valid.
     */
    bool getMessage(KraftMessage_Interface* kraftMessage, const bool &peek = false);

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @param kraftMessage is a pointer to a class that inherets from KraftMessage_Interface.
     * @param peek is a bool. If set to true then the message will not be removed from queue. Default value if false.
     * @returns true if message valid.
     */
    void removeMessage() {if (messageAvailable()) receivedPackets_.pop_back();}

    /**
     * Gets the nodes ID it uses when communicating with other transceivers
     * 
     * @return nodes ID.
     */
    uint16_t getSelfID() {return selfID_;}

    /**
     * Gives system time to do things.
     */
    void loop();


private:

    //Struct for storing packet information till picked up by application
    struct ReceivedPayloadData {

        uint8_t dataBuffer[255];

        MessageData messageData;

    };

    //Struct for storing packets that will be sent. 
    struct SendPacketData {

        uint8_t dataBuffer[255];
        uint32_t bufferSize = 0;

        uint8_t power_dB = 0;

        bool waitforAck = false;

        eKraftPacketNodeID_t receivingNodeID = eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast;

        uint32_t sendTimestamp = 0;
        uint8_t sendAttempts = 0;

        uint32_t sendInterval = 0;

    };

    //Struct to store information about a node
    struct NodeData {

        //Stores last packetcounter for counting amount of lost packets.
        uint8_t lastPacketCounter = 0;

        //Counter for all packets that have been lost.
        uint32_t packetsLost = 0;

        //Points to packet that its waiting for. Should be nullptr when not waiting
        SendPacketData* waitingOnPacket = nullptr;

    };

    //returns true when successfull. Ackrequested bool is an output and is true if an ack was requested by sender.
    bool decodeMessageFromBuffer(ReceivedPayloadData* payloadData, bool* ackRequested, uint8_t* buffer, uint32_t bufferSize);

    //Returns number of bytes written into buffer
    uint32_t encodeMessageToBuffer(KraftMessage_Interface* messagePointer, const uint8_t &receiveNode, const bool &requestAck, uint8_t* buffer, const uint32_t &bufferSize);

    //Calculate CRC of packet.
    uint8_t calculateCRC(uint8_t* buffer, const uint32_t &stopByte);


    KraftLink_Interface* dataLink_; //Link used for sending receiving packets

    //Queue containing all received packets.
    Circular_Buffer<ReceivedPayloadData, 10> receivedPackets_;
    //Queue containing all packets to send.
    Circular_Buffer<SendPacketData, 10> sendPackets_;
    //Queue containing all packets to send that require an ACK. These must wait until the one before it is finished
    Circular_Buffer<SendPacketData, 10> sendPacketsACK_;

    NodeData nodeData_[255];

    //Index corresponds to node that should receive packet.
    uint32_t sentPacketsCounter_[255];

    eKraftPacketNodeID_t selfID_;


};



#endif 
