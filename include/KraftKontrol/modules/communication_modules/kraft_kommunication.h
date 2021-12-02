#ifndef KRAFT_KOMMUNICATION_H
#define KRAFT_KOMMUNICATION_H



#include "Arduino.h"

#include "stdint.h"

#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/modules/module_abstract.h"
#include "KraftKontrol/utils/system_time.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

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

    //Maximum number of nodes on network
    const uint32_t c_maxNumberNodes = 255;

    //Rate at which heartbeat messages should be sent
    const uint32_t c_heartbeatRate = 4;

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

    uint32_t getMessageType() const {return eKraftMessageType_t::eKraftMessageType_Datalink_ID;}

    uint32_t getDataType() const {return eDataLinkDataType_t::eDataLinkDataType_Heartbeat;}

    uint32_t getDataSize() const {return 0;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {
        return true;
    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) {
        return true;
    }

};



class KraftMessageACK: public KraftMessage_Interface {
public:

    uint32_t getMessageType() const {return eKraftMessageType_t::eKraftMessageType_Datalink_ID;}

    uint32_t getDataType() const {return eDataLinkDataType_t::eDataLinkDataType_ACK;}

    uint32_t getDataSize() const {return 0;};

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {
        return true;
    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) {
        return true;
    }

};



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



/**
 * Used to store data of where data should be sent to or was received from and if it should be lossless.
 */
struct MessageLinkData {

    //Container for message.
    KraftMessageEmulator message;

    //What device sent this message.
    eKraftMessageNodeID_t transmitterID;
    //What device should receive this message.
    eKraftMessageNodeID_t receiverID;

    //Set to true for the system to wait for an ack after sending.
    bool requiresAck = false; 

    //Received data RSSI. Signifies signal strength but for LoRa SNR is better.
    uint8_t signalRSSI = 0;
    //Received data RSSI. Signifies signal strength in reference to noise. Better for LoRa.
    uint8_t signalSNR = 0;
    //What power in dB to send the message at.
    uint8_t sendPowerdB = 10;

};



class KraftKommunication: public Module_Abstract, public Task_Abstract {
public:

    /**
     * Creates a KraftKommunication object to handle a radio object and its packets
     * 
     * @param dataLink is a radio class that inherets from KraftLink_Interface that will be used to send and receive packets 
     */
    KraftKommunication(KraftLink_Abstract& dataLink, eKraftMessageNodeID_t selfID): Task_Abstract("Kraft Kommunication", 500, eTaskPriority_t::eTaskPriority_Middle), dataLink_(dataLink) {
        selfID_ = selfID;
        receivedPacketSub_.subscribe(dataLink.getReceivedDataTopic());
        dataLinkSendSubr_.subscribe(dataLink.getToSendDataTopic());

        sendBroadcastMessageSubr_.subscribe(sendBroadcastMessageTopic_);
        sendBroadcastMessageSubr_.setCallbackObject(this);
        sendBroadcastMessageSubr_.setCallbackFunction(&KraftKommunication::kraftMessageBroadcastCallback);

        sendMessageSubr_.subscribe(sendMessageTopic_);
        sendMessageSubr_.setCallbackObject(this);
        sendMessageSubr_.setCallbackFunction(&KraftKommunication::kraftMessageSendCallback);

    }

    /**
     * Sends the given message.
     * 
     * @param kraftMessage is a KraftMessage_Interface class containing the data to be sent.
     * @returns false if queue is full. loop() needs to be ran to give link time to send data and make room.
     */
    bool sendMessage(const KraftMessage_Interface& kraftMessage, eKraftMessageNodeID_t receiveNodeID, bool requiresAck = false);

    /**
     * Checks if node is online.
     * Offline status usually is checked if no message was received from a node for 500ms.
     * @param node Node to check.
     * @returns node status
     */
    bool getNodeStatus(eKraftMessageNodeID_t node) {return nodeData_[constrain(node, 0, 255)].online;}

    /**
     * @returns topic to which received messages are published.
     */
    const Topic<KraftMessage_Interface>& getReceivedMessageTopic() const {return receivedMessagesTopic_;}

    /**
     * @returns topic to which messages can be published that will be broadcasted to network.
     */
    Topic<KraftMessage_Interface>& getBroadcastMessageTopic() {return sendBroadcastMessageTopic_;}

    /**
     * @returns topic to which messages can be published that will be sent to network with user specified settings.
     */
    Topic<MessageLinkData>& getSendMessageTopic() {return sendMessageTopic_;}

    /**
     * Gets the nodes ID it uses when communicating with other transceivers
     * 
     * @return nodes ID.
     */
    uint16_t getSelfID() {return selfID_;}

    /**
     * Gives system time to do things.
     */
    void thread();


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
        int64_t lastPacketTimestamp = 0;

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
    uint32_t encodeMessageToBuffer(const KraftMessage_Interface& message, uint8_t receiveNode, bool requestAck, uint8_t* buffer, uint32_t bufferSize);

    //Calculate CRC of packet.
    uint8_t calculateCRC(uint8_t* buffer, uint32_t stopByte);

    //Sends kraftmessage to data link
    void kraftMessageSendCallback(const MessageLinkData& item);
    //Sends kraftmessage to data link
    void kraftMessageBroadcastCallback(const KraftMessage_Interface& item);


    //Link used for sending and receiving packets
    KraftLink_Abstract& dataLink_;

    //Topic that gets messages published to when messages are received.
    Topic<KraftMessage_Interface> receivedMessagesTopic_;

    //Buffer for data packets received by data link
    Buffer_Subscriber<DataMessageBuffer, 50> receivedPacketSub_;
    //Subscriber for data packets to be published to datalink
    Simple_Subscriber<DataMessageBuffer> dataLinkSendSubr_;

    //Topic to which messages with user specified settings can be sent to network.
    Topic<MessageLinkData> sendMessageTopic_;
    Callback_Subscriber<MessageLinkData, KraftKommunication> sendMessageSubr_;

    //Topic to which messages can be published and automatically broadcasted.
    Topic<KraftMessage_Interface> sendBroadcastMessageTopic_;
    Callback_Subscriber<KraftMessage_Interface, KraftKommunication> sendBroadcastMessageSubr_;


    //Buffer containing messages to be sent but without ack
    Buffer<SendPacketData, 10> sendPackets_;
    //Buffer containing messages to be sent but with ack.
    Buffer<SendPacketData, 10> sendPacketsACK_;


    NodeData nodeData_[c_maxNumberNodes];

    //Index corresponds to node that should receive packet.
    uint32_t sentPacketsCounter_[c_maxNumberNodes];

    eKraftMessageNodeID_t selfID_;

    //Timer to keep track of sending heatbeat messages
    IntervalControl heartbeatTimer_ = IntervalControl(c_heartbeatRate);


};



#endif 
