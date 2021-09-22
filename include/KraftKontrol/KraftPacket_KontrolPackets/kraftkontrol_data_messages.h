#ifndef KRAFTKONTROL_DATA_MESSAGES_H
#define KRAFTKONTROL_DATA_MESSAGES_H



#include "kraftkontrol_messagetype_abstracts.h"



enum eMessageTypeData_t: uint32_t {
    eMessageTypeData_AngularAcceleration,
    eMessageTypeData_AngularVelocity,
    eMessageTypeData_Attitude,
    eMessageTypeData_Acceleration,
    eMessageTypeData_Velocity,
    eMessageTypeData_Position,
    eMessageTypeData_FullKinematics,
    eMessageTypeData_Force,
    eMessageTypeData_Torque,
    eMessageTypeData_FullDynamics,
    ///Current vehicle mode status.
    eMessageTypeData_VehicleModeSet,
    ///Tells vehicle to run a program at a certain time.
    eMessageTypeData_ProgramStart,
    ///Basestation rc channels sent to vehicle.
    eMessageTypeData_RCChannels,
    ///A simple buffer containing raw data. (Might be deprecated in future)
    eMessageTypeData_Buffer,
    ///Current node status change. Meaning if node connected or was disconnected. Usually published onto global module topic.
    eMessageTypeData_NodeStatus
};



class DataMessageAttitude: public MessageQuaternion_Abstract {
public:

    DataMessageAttitude() {}

    DataMessageAttitude(const Quaternion<> &attitude): MessageQuaternion_Abstract(attitude) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_Attitude;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessagePosition: public MessageVector_Abstract {
public:

    DataMessagePosition() {}

    DataMessagePosition(const Vector<> &position): MessageVector_Abstract(position) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_Position;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageFullKinematics: public MessageFullKinematics_Abstract {
public:

    DataMessageFullKinematics() {}

    DataMessageFullKinematics(const KinematicData &kinematics): MessageFullKinematics_Abstract(kinematics) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_FullKinematics;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageForce: public MessageVector_Abstract {
public:

    DataMessageForce() {}

    DataMessageForce(const Vector<> &force): MessageVector_Abstract(force) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_Force;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageTorque: public MessageVector_Abstract {
public:

    DataMessageTorque() {}

    DataMessageTorque(const Vector<> &torque): MessageVector_Abstract(torque) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_Torque;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageNodeStatus: public KraftMessage_Interface {
public:

    DataMessageNodeStatus() {}

    /**
     * @param nodeConnected Whether or not node is connected.
     */
    DataMessageNodeStatus(bool nodeConnected, uint32_t nodeID) {
        nodeConnected_ = nodeConnected;
        nodeID_ = nodeID;
    }

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_NodeStatus;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

    /**
     * @returns true if node is connected to network.
     */
    const bool& getNodeConnected() const {return nodeConnected_;}

    /**
     * @returns true if node is connected to network.
     */
    const uint32_t& getNodeID() const {return nodeID_;}


    uint32_t getDataSize() const {return sizeof(nodeConnected_) + sizeof(nodeID_);}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&nodeID_, sizeof(nodeID_));
        bufferWrite(&nodeConnected_, sizeof(nodeConnected_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&nodeID_, sizeof(nodeID_));
        bufferRead(&nodeConnected_, sizeof(nodeConnected_));
        endBufferRead();

        return true;

    }

private:

    bool nodeConnected_ = false;
    uint32_t nodeID_ = 0;

};  



/**
 * Stores raw bytes for data transfer.
 */
class DataMessageBuffer: public KraftMessage_Interface{
public:

    DataMessageBuffer() {}

    DataMessageBuffer(uint8_t* buffer, uint32_t bufferSize) {
        setBuffer(buffer, bufferSize);
    }

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_Buffer;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

    uint32_t getDataSize() const {return getBufferSize() + sizeof(bufferSize_);}

    bool setBuffer(uint8_t* buffer, uint32_t bufferSize) {
        if (bufferSize > KraftMessage::c_messageContainerArraySize_) return false;
        bufferSize_ = bufferSize;
        for (uint32_t i = 0; i < bufferSize; i++) buffer_[i] = buffer[i];
        return true;
    }

    void setBufferSize(uint32_t size) {bufferSize_ = size;} 

    uint8_t* getBuffer() {return buffer_;}
    uint32_t getBufferSize() const {return bufferSize_;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&bufferSize_, sizeof(bufferSize_));
        for (uint8_t i = 0; i < bufferSize_; i++) bufferWrite(&buffer_[i], sizeof(buffer_[0]));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&bufferSize_, sizeof(bufferSize_));
        for (uint8_t i = 0; i < bufferSize_; i++) bufferRead(&buffer_[i], sizeof(buffer_[0]));
        endBufferRead();

        return true;

    }


protected:

    uint8_t buffer_[KraftMessage::c_messageContainerArraySize_];
    uint32_t bufferSize_ = 0;


};



class DataMessageFullDynamics: public MessageFullDynamics_Abstract {
public:

    DataMessageFullDynamics() {}

    DataMessageFullDynamics(const DynamicData &dynamics): MessageFullDynamics_Abstract(dynamics) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_FullDynamics;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageVehicleModeSet: public MessageGeneric_Abstract<eVehicleMode_t> {
public:

    DataMessageVehicleModeSet() {}

    DataMessageVehicleModeSet(const eVehicleMode_t &vehicleMode): MessageGeneric_Abstract<eVehicleMode_t>(vehicleMode) {}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_VehicleModeSet;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

};



class DataMessageRCChannels: public KraftMessage_Interface{
public:

    DataMessageRCChannels() {}

    /**
     * Constructor for array as input
     * @param channels Pointer to a int16_t array.
     * @param numChannels Number of channels to copy from channels.
     */
    DataMessageRCChannels(int16_t* channels, const uint8_t &numChannels) {
        for (uint8_t i = 0; i < numChannels && i < c_maxChannels; i++) channels_[i] = channels[i];
    }

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_RCChannels;}

    uint32_t getDataSize() const {return sizeof(channels_);}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

    int16_t getChannel(const uint8_t &channel) {return channels_[channel];}

    void getChannelAll(int16_t* channelArray, uint8_t numChannels = c_maxChannels) {for (uint8_t i = 0; i < numChannels; i++) channelArray[i] = channels_[i];}

    void setChannel(const int16_t &value, const uint8_t &channel) {channels_[channel] = value;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        for (uint8_t i = 0; i < c_maxChannels; i++) bufferWrite(channels_, sizeof(channels_[0]));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        for (uint8_t i = 0; i < c_maxChannels; i++) bufferRead(channels_, sizeof(channels_[0]));
        endBufferRead();

        return true;

    }


protected:

    static const uint8_t c_maxChannels = 15;

    int16_t channels_[c_maxChannels];

};


class DataMessageProgramStart: public KraftMessage_Interface{
public:

    DataMessageProgramStart() {}

    DataMessageProgramStart(uint32_t programID, int64_t programStartTime) {
        programID_ = programID;
        programStartTime_ = programStartTime;
    }

    void setProgramID(uint32_t programID) {programID_ = programID;} 
    uint32_t getProgramID() const {return programID_;}

    void setProgramStartTime(uint32_t programStartTime) {programStartTime_ = programStartTime;} 
    int64_t getProgramStartTime() const {return programStartTime_;}

    uint32_t getDataType() const final {return eMessageTypeData_t::eMessageTypeData_ProgramStart;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Data_ID;}

    uint32_t getDataSize() const {return sizeof(programID_) + sizeof(programStartTime_);}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&programID_, sizeof(programID_));
        bufferWrite(&programStartTime_, sizeof(programStartTime_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&programID_, sizeof(programID_));
        bufferRead(&programStartTime_, sizeof(programStartTime_));
        endBufferRead();

        return true;

    }


protected:

    uint32_t programID_ = 0;
    int64_t programStartTime_ = 0;

};


/*
class KraftMessageMagCalValuesIs:{
public:

    KraftMessageMagCalValuesIs() {}

    KraftMessageMagCalValuesIs(const Vector<> &magMax, const Vector<> &magMin) {
        magMin_ = magMin;
        magMax_ = magMax;
    }

    uint32_t getDataType() const final {return eMessageTypeData_t::eKraftMessageType_KraftKontrol_MagCalibIs;}

    uint32_t getDataSize() const {return sizeof(magMin_) + sizeof(magMax_);}

    const Vector<>& getMinValue() const {return magMin_;}
    const Vector<>& getMaxValue() const {return magMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {magMin_ = magMin; magMax_ = magMax;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&magMin_.x, sizeof(magMin_.x));
        bufferWrite(&magMin_.y, sizeof(magMin_.y));
        bufferWrite(&magMin_.z, sizeof(magMin_.z));
        bufferWrite(&magMax_.x, sizeof(magMax_.x));
        bufferWrite(&magMax_.y, sizeof(magMax_.y));
        bufferWrite(&magMax_.z, sizeof(magMax_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) {

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&magMin_.x, sizeof(magMin_.x));
        bufferRead(&magMin_.y, sizeof(magMin_.y));
        bufferRead(&magMin_.z, sizeof(magMin_.z));
        bufferRead(&magMax_.x, sizeof(magMax_.x));
        bufferRead(&magMax_.y, sizeof(magMax_.y));
        bufferRead(&magMax_.z, sizeof(magMax_.z));
        endBufferRead();

        return true;

    }


protected:

    Vector<> magMin_ = 90000000;
    Vector<> magMax_ = -90000000;

};



class KraftMessageMagCalValuesSet:{
public:

    KraftMessageMagCalValuesSet() {}

    KraftMessageMagCalValuesSet(const Vector<> &magMax, const Vector<> &magMin) {
        magMin_ = magMin;
        magMax_ = magMax;
    }

    uint32_t getDataType() const final {return eMessageTypeData_t::eKraftMessageType_KraftKontrol_MagCalibSet;}

    uint32_t getDataSize() const {return sizeof(magMin_) + sizeof(magMax_);}

    const Vector<>& getMinValue() const {return magMin_;}
    const Vector<>& getMaxValue() const {return magMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {magMin_ = magMin; magMax_ = magMax;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&magMin_.x, sizeof(magMin_.x));
        bufferWrite(&magMin_.y, sizeof(magMin_.y));
        bufferWrite(&magMin_.z, sizeof(magMin_.z));
        bufferWrite(&magMax_.x, sizeof(magMax_.x));
        bufferWrite(&magMax_.y, sizeof(magMax_.y));
        bufferWrite(&magMax_.z, sizeof(magMax_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) {

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&magMin_.x, sizeof(magMin_.x));
        bufferRead(&magMin_.y, sizeof(magMin_.y));
        bufferRead(&magMin_.z, sizeof(magMin_.z));
        bufferRead(&magMax_.x, sizeof(magMax_.x));
        bufferRead(&magMax_.y, sizeof(magMax_.y));
        bufferRead(&magMax_.z, sizeof(magMax_.z));
        endBufferRead();

        return true;

    }


protected:

    Vector<> magMin_ = 90000000;
    Vector<> magMax_ = -90000000;

};*/



#endif 
