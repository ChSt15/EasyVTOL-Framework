#ifndef KRAFT_MESSAGE_H
#define KRAFT_MESSAGE_H



#include "stdint.h"

#include "string.h"



/**
 * Indicates the type of message. 
 * This is to differentiate between telemetry, telecommands, general data etc.
 * Use eKraftMessageType_End_ID to extend the enum. MUST BE AT END OF ENUM!
 */
enum eKraftMessageType_t: uint32_t {
    //Used for initialising and signify that it was never set.
    eKraftMessageType_Invalid_ID,
    //General data.
    eKraftMessageType_Data_ID,
    //Telemetry data.
    eKraftMessageType_Telemetry_ID,
    //Telecommand data.
    eKraftMessageType_Telecommand_ID,
    //Datalink information e.g. ack, radio setting changes etc.
    eKraftMessageType_Datalink_ID,
    //This is to extend the enum
    eKraftMessageType_End_ID
};



class KraftMessage_Interface {
public: 

    /**
     * Message type is to differentiate between telemetry, telecommands, and general data.
     * @returns the type of message. Not the type of data its holding.
     */
    virtual uint32_t getMessageType() const = 0;

    /**
     * Used to find out what datatype we are dealing with.
     * @returns ID of datatype
     */
    virtual uint32_t getDataType() const = 0;

    /**
     * @returns size of data in bytes
     */
    virtual uint32_t getDataSize() const = 0;

    /**
     * Places the contents of the message into the given buffer at the requested start position.
     * @param dataBytes Pointer to the buffer the will receive the raw data.
     * @param dataByteSize Max size of the given buffer to make sure given buffer is large enough.
     * @param startByte Index of the starting position to place raw data. Defaults to 0.
     * @returns true if successfull, false if something was not right.
     */
    virtual bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const = 0;

    /**
     * Places the contents of the message into the given buffer at the requested start position.
     * @param dataBytes Pointer to the buffer that containes the raw data.
     * @param dataByteSize Size of the given buffer to make sure given data if correct.
     * @param startByte Index of the starting position to take data from and increment from.
     * @returns true if successfull, false if something was not right.
     */
    virtual bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) = 0;

private:

    mutable uint32_t bufferIndex = 0;
    mutable uint32_t bufferMaxIndex;
    mutable void* bufferWritePointer = nullptr;
    const void* bufferReadPointer = nullptr;

protected:


    /**
     * Call this before using bufferWrite(..). 
     * @param pointerToBuffer Pointer to buffer where all following data will be written.
     * @param startIndex Index of buffer to start reading from. Default 0.
     * @param maxIndex Index of buffer to limit reading. Default max of uint32_t.
     */
    void startBufferWrite(void* pointerToBuffer, const uint32_t &startIndex = 0, const uint32_t &maxIndex = UINT32_MAX) const {bufferIndex = startIndex; bufferMaxIndex = maxIndex; bufferWritePointer = pointerToBuffer;}

    /**
     * Call this after using bufferWrite(..). 
     */
    void endBufferWrite() const {bufferIndex = 0; bufferWritePointer = nullptr;}

    /**
     * Used to make copy lots of data easier. 
     * Call startBufferWrite(...) before copying and endBufferWrite() after copying all data.
     * @param data Pointer to the data to be copied
     * @param numberBytes Number of bytes to be copied. Simply use sizeof("data type to be copied").
     * @returns true if copied, false if failure.
     */
    bool bufferWrite(const void* data, const uint32_t &numberBytes) const {

        if (bufferWritePointer == nullptr || bufferIndex + numberBytes >= bufferMaxIndex) return false;

        memcpy((uint8_t*)bufferWritePointer + bufferIndex, data, numberBytes);
        bufferIndex += numberBytes;

        return true;

    }

    /**
     * Call this before using bufferRead(..). 
     * @param pointerToBuffer Pointer to buffer where all following data will be read.
     * @param startIndex Index of buffer to start writting to. Default 0.
     * @param maxIndex Index of buffer to limit writting. Default max of uint32_t.
     */
    void startBufferRead(const void* pointerToBuffer, const uint32_t &startIndex = 0, const uint32_t &maxIndex = UINT32_MAX) {bufferIndex = startIndex; bufferMaxIndex = maxIndex; bufferReadPointer = pointerToBuffer;}

    /**
     * Call this after using bufferWrite(..). 
     */
    void endBufferRead() {bufferIndex = 0; bufferReadPointer = nullptr; bufferIndex = 0; bufferMaxIndex = UINT32_MAX;}

    /**
     * Used to make copy lots of data easier. 
     * Call startBufferWrite(...) before copying and endBufferWrite() after copying all data.
     * @param data Pointer to the data to be copied
     * @param numberBytes Number of bytes to be copied. Simply use sizeof("data type to be copied").
     * @returns true if copied, false if failure.
     */
    bool bufferRead(void* data, const uint32_t &numberBytes) {

        if (bufferReadPointer == nullptr || bufferIndex + numberBytes >= bufferMaxIndex) return false;

        memcpy(data, (uint8_t*)bufferReadPointer + bufferIndex, numberBytes);
        bufferIndex += numberBytes;

        return true;

    }

};


/**
 * Base class to simplify creating Data messages. This automatically implements getMessageType().
 */
class KraftMessageData_Abstract: public KraftMessage_Interface {
public:

    /**
     * Message type is to differentiate between telemetry, telecommands, and general data.
     * @returns the type of message. Not the type of data its holding.
     */
    virtual uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class KraftMessageTelemetry_Abstract: public KraftMessage_Interface {
public:

    /**
     * Message type is to differentiate between telemetry, telecommands, and general data.
     * @returns the type of message. Not the type of data its holding.
     */
    virtual uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class KraftMessageTelecommand_Abstract: public KraftMessage_Interface {
public:

    /**
     * Message type is to differentiate between telemetry, telecommands, and general data.
     * @returns the type of message. Not the type of data its holding.
     */
    virtual uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telecommand_ID;}

};



class KraftMessageDatalink_Abstract: public KraftMessage_Interface {
public:

    /**
     * Message type is to differentiate between telemetry, telecommands, and general data.
     * @returns the type of message. Not the type of data its holding.
     */
    virtual uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Datalink_ID;}

};



/**
 * This can be used to copy any type of KraftMessage and store it.
 */
class KraftMessageContainer {
public:

    KraftMessageContainer() {}

    /**
     * @param message Reference to message.
     */
    KraftMessageContainer(const KraftMessage_Interface& message) {
        setMessage(message);
    }

    /**
     * Used to store the data directly even if its an unknown data type.
     * @param dataBytes Pointer to buffer containing data.
     * @param numberBytes Exact number of raw bytes to contain.
     * @param messageTypeID ID of message type.
     * @param dataTypeID ID of data type.
     */
    KraftMessageContainer(const uint8_t* dataBytes, uint32_t numberBytes, uint32_t messageTypeID, uint32_t dataTypeID) {
        setMessage(dataBytes, numberBytes, dataTypeID, messageTypeID);
    }

    ~KraftMessageContainer() {delete[] data_;}

    /**
     * @param message Reference to message.
     * @returns true if successfull.
     */
    inline bool setMessage(const KraftMessage_Interface& message) {
        delete[] data_;
        dataType_ = message.getDataType();
        dataSize_ = message.getDataSize();
        messageType_ = message.getMessageType();
        data_ = new uint8_t[dataSize_];
        if (message.getRawData(data_, dataSize_)) {
            isValid_ = true;
            return true;
        }
        isValid_ = false;
        return false;

    }

    /**
     * Used to store the data directly even if its an unknown data type.
     * @param dataBytes Pointer to buffer containing data.
     * @param numberBytes Exact number of raw bytes to contain.
     * @param messageTypeID ID of message type.
     * @param dataTypeID ID of data type.
     * @returns true if successfull.
     */
    inline bool setMessage(const uint8_t* dataBytes, const uint32_t& numberBytes, const uint32_t& dataTypeID, const uint32_t& messageTypeID) {
        delete[] data_;
        dataType_ = dataTypeID;
        dataSize_ = numberBytes;
        messageType_ = messageTypeID;
        data_ = new uint8_t[dataSize_];
        for (uint32_t i = 0; i < dataSize_;i++) data_[i] = dataBytes[i];
        isValid_ = true;
        return true;
    }

    inline const uint32_t& getDataType() const {return dataType_;}

    inline uint32_t getMessageType() const {return messageType_;}

    inline const bool& isValid() const {return isValid_;}

    inline bool getMessage(KraftMessage_Interface& message) const {
        if (!isValid_) return false;
        return message.setRawData(data_, dataSize_);
    }

    inline KraftMessageContainer& operator= (const KraftMessageContainer& a) {
        setMessage(a.data_, a.dataSize_, a.dataType_, a.messageType_);
        return *this;
    }

private:

    uint8_t* data_ = nullptr;
    uint32_t dataType_ = 0;
    uint32_t messageType_ = eKraftMessageType_t::eKraftMessageType_Invalid_ID;
    uint8_t dataSize_ = 0;

    bool isValid_ = false;

};



#endif 
