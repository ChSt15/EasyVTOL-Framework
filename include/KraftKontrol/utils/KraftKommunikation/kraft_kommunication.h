#ifndef KRAFT_KOMMUNICATION_H
#define KRAFT_KOMMUNICATION_H



#include "Arduino.h"

#include "stdint.h"

#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/modules/module_abstract.h"
#include "KraftKontrol/utils/system_time.h"

#include "kraft_link.h"
#include "kraft_message.h"


/**
 * Packet Structure:
 * n is the size if the packets data.
 * - First byte 0 and 1 are the Kraftpacket startbyte markers. These must be first checked to make sure the Packet is actually a kraftpacket and correct version
 * - byte 2 is the transmitterID
 * - byte 3 is the receiverID. A packet not adressed to a receiver should be ignored.
 * - byte 4 indicates what the message type is.
 * - byte 5 indicates what the data type is.
 * - byte 6 is a flag for if an ACK packet must be returned.
 * - byte 7 is the packet incrementer. This is incremented for every new byte addressed to a specific reciever. Used for counting lost packets.
 * - byte 8 is the size of the packets payload (n ist this number).
 * - byte 9 to (8+n)th byte is the raw data
 * - (9+n)th byte is the crc byte to check if data is valid
 * - (10+n) is the endbyte marker
 */



namespace {

    const uint16_t c_kraftPacketStartMarker = 0xBD6E;
    const uint8_t c_kraftPacketEndMarker = 0xDE;

    const uint16_t c_kraftPacketVersion = 0; //Should be changed when something in the protocoll was changed.

    //Number of times a packet will be sent if an ack is required.
    const uint8_t c_sendAttempts = 10;
    //Amount of microseconds to wait for an ack before resending after the link says the packet was sent, not when given the packet.
    const uint32_t c_sendTimeout = 500; 
    //Default power to send packets at.
    const uint8_t c_sendPowerDefault = 10;

    enum eDataLinkDataType_t: uint8_t {
        eDataLinkDataType_Heartbeat = 0,
        eDataLinkDataType_ACK
    };

}  




//Is the ID of a devide like Vehicle or basestation. Broadcast means the data is meant for all devices to receive. Also used for Transmitting device ID.
enum eKraftMessageNodeID_t : uint8_t {
    eKraftMessageNodeID_vehicle = 1,                //Vehicle ID
    eKraftMessageNodeID_controller = 2,             //Controllers ID. Controller is for manual control
    eKraftMessageNodeID_basestation = 3,            //Basestation ID. 
    eKraftMessageNodeID_broadcast = 255             //General packet that everything can receive. Basically a broadcast.
};



class KraftMessageHeartbeat: public KraftMessage_Interface {
public:

    uint32_t getMessageType() const {return eKraftMessageType_t::eKraftMessageType_DataLink_ID;}

    uint32_t getDataType() const {{return eDataLinkDataType_t::eDataLinkDataType_Heartbeat;}

    uint32_t getDataSize() const {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {
        return true;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

};



class KraftMessageACK: public KraftMessage_Interface {
public:

    uint32_t getMessageType() const {return eKraftMessageType_t::eKraftMessageType_DataLink_ID;}

    uint32_t getDataTypeID() const {return eDataLinkDataType_t::eDataLinkDataType_ACK;}

    uint32_t getDataSize() const {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {
        return true;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

};


/*
class KraftMessageStringPacket final: public KraftMessage_Interface {
public:

    KraftMessageStringPacket() {}

    KraftMessageStringPacket(const char string[]) {
        setString(string);
    }

    ~KraftMessageStringPacket() {
        delete[] stringPointer;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_t::eKraftMessageType_String_ID;}

    uint32_t getDataSize() const {return sizeStringPointer;};

    uint32_t getStringLength() const {return sizeStringPointer;}

    bool getString(char string[], const uint32_t &sizeStringDest) const {

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

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        return getString((char*)dataBytes+startByte, dataByteSize);

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        setString((char*)(dataBytes)+startByte);

        return true;

    }


private:

    char* stringPointer = nullptr;
    uint32_t sizeStringPointer = 0;

};*/



struct MessageData {

    eKraftMessageNodeID_t transmitterID;
    eKraftMessageNodeID_t receiverID;
    uint8_t messageTypeID = 0;
    uint8_t dataTypeID = 0;
    uint8_t payloadSize = 0;
    uint8_t messageCounter = 0;

    uint8_t signalRSSI = 0;
    uint8_t signalSNR = 0;

};



class KraftKommunication: public Module_Abstract {
public:

    /**
     * Creates a KraftKommunication object to handle a radio object and its packets
     * 
     * @param dataLink is a radio class that inherets from KraftLink_Interface that will be used to send and receive packets 
     */
    KraftKommunication(KraftLink_Interface* dataLink, eKraftMessageNodeID_t selfID) {
        dataLink_ = dataLink;
        selfID_ = selfID;
    }

    /**
     * Sends the given message.
     * 
     * @param kraftMessage is a KraftMessage_Interface class containing the data to be sent.
     * @returns false if queue is full. loop() needs to be ran to give link time to send data and make room.
     */
    bool sendMessage(KraftMessage_Interface& kraftMessage, const eKraftMessageNodeID_t &receiveNodeID, const bool &requiresAck = false);

    /**
     * Checks if a message is available.
     * 
     * @returns Number of messages available
     */
    uint16_t messageAvailable() {return receivedPackets_.available();}

    /**
     * Checks if transmitter buffers are full. Sending a packet when buffers are full will fail or erase oldest message in buffer waiting to be sent.
     * 
     * @see networkAckBusy() for ack buffer
     * @returns Whether the buffer is full
     */
    bool networkBusy() {return sendPackets_.availableSpace() == 0;}

    /**
     * Checks if transmitter buffers are full. Sending a packet when buffers are full will fail or erase oldest message in buffer waiting to be sent.
     * 
     * @see networkBusy() for nona ack buffer
     * @returns Whether the buffer is full
     */
    bool networkAckBusy() {return sendPacketsACK_.availableSpace() == 0;}

    /**
     * Checks if node is online.
     * Offline status usually is checked if no message was received from a node for 500ms.
     * @param node Node to check.
     * @returns node status
     */
    bool getNodeStatus(eKraftMessageNodeID_t node) {return nodeData_[constrain(node, 0, 255)].online;}

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @returns MessageData struct.
     */
    MessageData getMessageInformation() {
        ReceivedPayloadData buf;
        receivedPackets_.peekBack(&buf);
        return buf.messageData;
    };

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @param kraftMessage is a pointer to a class that inherets from KraftMessage_Interface.
     * @param peek is a bool. If set to true then the message will not be removed from queue. Default value if false.
     * @returns true if message valid.
     */
    bool getMessage(KraftMessage_Interface& kraftMessage, const bool &peek = false);

    /**
     * Returns latest received message data. Use this to find out what KraftMessage_Interface class needs to be given.
     * 
     * @param kraftMessage is a pointer to a class that inherets from KraftMessage_Interface.
     * @param peek is a bool. If set to true then the message will not be removed from queue. Default value if false.
     * @returns true if message valid.
     */
    void removeMessage() {receivedPackets_.removeBack();}

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

        eKraftMessageNodeID_t receivingNodeID = eKraftMessageNodeID_t::eKraftMessageNodeID_broadcast;

        uint32_t sendTimestamp = 0;
        uint8_t sendAttempts = 0;

        uint32_t sendInterval = 0;

    };

    //Struct to store information about a node
    struct NodeData {

        //Stores the last time a packet was received. Used to see if still connected.
        uint32_t lastPacketTimestamp = 0;

        //Whether the node is online. 
        bool online = false;

        //Stores last packetcounter for counting amount of lost packets.
        uint8_t lastPacketCounter = 0;

        //Counter for all packets that have been lost.
        uint32_t packetsLost = 0;

        //Points to packet that its waiting for. Should be nullptr when not waiting
        SendPacketData* waitingOnPacket = nullptr;

    };

    //returns true when successfull. Ackrequested bool is an output and is true if an ack was requested by sender.
    bool decodeMessageFromBuffer(ReceivedPayloadData& payloadData, bool& ackRequested, uint8_t* buffer, uint32_t bufferSize);

    //Returns number of bytes written into buffer
    uint32_t encodeMessageToBuffer(KraftMessage_Interface& message, const uint8_t &receiveNode, const bool &requestAck, uint8_t* buffer, const uint32_t &bufferSize);

    //Calculate CRC of packet.
    uint8_t calculateCRC(uint8_t* buffer, const uint32_t &stopByte);


    KraftLink_Interface* dataLink_; //Link used for sending receiving packets

    //Queue containing all received packets.
    Buffer<ReceivedPayloadData, 10> receivedPackets_;
    //Queue containing all packets to send.
    Buffer<SendPacketData, 10> sendPackets_;
    //Queue containing all packets to send that require an ACK. These must wait until the one before it is finished
    Buffer<SendPacketData, 10> sendPacketsACK_;

    NodeData nodeData_[255];

    //Index corresponds to node that should receive packet.
    uint32_t sentPacketsCounter_[255];

    eKraftMessageNodeID_t selfID_;


};



#endif 
