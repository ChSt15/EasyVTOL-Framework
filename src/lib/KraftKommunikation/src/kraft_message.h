#ifndef KRAFT_MESSAGE_H
#define KRAFT_MESSAGE_H



#include "stdint.h"

#include "string.h"



//Indicates the type of packet. Use eKraftMessageType_StandardEnd_ID and up for custom Packets
enum eKraftMessageType_t : uint8_t {
    //Simply signals that connection is still there. Should be send at 1Hz rate.
    eKraftMessageType_Heartbeat_ID,
    //To be returned to signal succesfull receive when requiested.
    eKraftMessageType_Ack_ID,
    //Generic packet with parameters for updating RadioSettings.
    eKraftMessageType_RadioSettings_ID,
    //Used for sending strings.
    eKraftMessageType_String_ID,
    //Use numbers higher than this for custom IDs
    eKraftMessageType_StandardEnd_ID
};



class KraftMessage_Interface {
public:

    virtual uint32_t getDataTypeID() = 0;

    virtual uint32_t getDataSize() = 0;

    virtual bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) = 0;

    virtual bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) = 0;

protected:

    uint32_t bufferIndex = 0;
    uint32_t bufferMaxIndex;
    void* bufferWritePointer = nullptr;
    const void* bufferReadPointer = nullptr;


    /**
     * Call this before using bufferWrite(..). 
     * @param pointerToBuffer Pointer to buffer where all following data will be written.
     * @param startIndex Index of buffer to start reading from. Default 0.
     * @param maxIndex Index of buffer to limit reading. Default max of uint32_t.
     */
    void startBufferWrite(void* pointerToBuffer, const uint32_t &startIndex = 0, const uint32_t &maxIndex = UINT32_MAX) {bufferIndex = startIndex; bufferMaxIndex = maxIndex; bufferWritePointer = pointerToBuffer;}

    /**
     * Call this after using bufferWrite(..). 
     */
    void endBufferWrite() {bufferIndex = 0; bufferWritePointer = nullptr;}

    /**
     * Used to make copy lots of data easier. 
     * Call startBufferWrite(...) before copying and endBufferWrite() after copying all data.
     * @param data Pointer to the data to be copied
     * @param numberBytes Number of bytes to be copied. Simply use sizeof("data type to be copied").
     * @returns true if copied, false if failure.
     */
    bool bufferWrite(void* data, const uint32_t &numberBytes) {

        if (bufferWritePointer == nullptr || bufferIndex + numberBytes >= bufferMaxIndex) return false;

        memcpy(bufferWritePointer + bufferIndex, data, numberBytes);
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

        memcpy(data, bufferReadPointer + bufferIndex, numberBytes);
        bufferIndex += numberBytes;

        return true;

    }

};



class KraftMessageHeartbeat final: public KraftMessage_Interface {
public:

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_Heartbeat_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

};



class KraftMessageACK final: public KraftMessage_Interface {
public:

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_Ack_ID;}

    uint32_t getDataSize() {return 0;};

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {
        return true;
    }

};



class KraftMessageStringPacket final: public KraftMessage_Interface {
public:

    KraftMessageStringPacket() {}

    KraftMessageStringPacket(const char string[]) {
        setString(string);
    }

    ~KraftMessageStringPacket() {
        delete[] stringPointer;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_t::eKraftMessageType_String_ID;}

    uint32_t getDataSize() {return sizeStringPointer;};

    uint32_t getStringLength() {return sizeStringPointer;}

    bool getString(char string[], const uint32_t &sizeStringDest) {

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

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        return getString((char*)dataBytes+startByte, dataByteSize);

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        setString((char*)(dataBytes)+startByte);

        return true;

    }


private:

    char* stringPointer = nullptr;
    uint32_t sizeStringPointer = 0;

};



#endif 
