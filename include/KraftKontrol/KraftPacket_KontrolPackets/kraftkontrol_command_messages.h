#ifndef KRAFTKONTROL_COMMAND_MESSAGES_H
#define KRAFTKONTROL_COMMAND_MESSAGES_H



#include "KraftKontrol/modules/communication_modules/kraft_message.h"
#include "kraftkontrol_messagetype_abstracts.h"

#include "lib/MathHelperLibrary/vector_math.h"
#include "lib/MathHelperLibrary/FML.h"

#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/vehicle_data.h"



enum eMessageTypeCommand_t : uint32_t {
    eKraftMessageType_KraftKontrol_Attitude,
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_VehicleMode,
    eKraftMessageType_KraftKontrol_RCChannels,
    eKraftMessageType_KraftKontrol_ProgramStart,
    eKraftMessageType_KraftKontrol_MagCalibData,
    eKraftMessageType_KraftKontrol_AccelCalibData,
    eKraftMessageType_KraftKontrol_GyroCalibData,
    eKraftMessageType_KraftKontrol_MagMountTransform,
    eKraftMessageType_KraftKontrol_AccelMountTransform,
    eKraftMessageType_KraftKontrol_GyroMountTransform
};



class CommandMessageAttitudeSet: public MessageQuaternion_Abstract {
public:

    CommandMessageAttitudeSet() {}

    CommandMessageAttitudeSet(const FML::Quaternion<> &attitude): MessageQuaternion_Abstract(attitude) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_Attitude;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessagePositionSet: public MessageVector_Abstract {
public:

    CommandMessagePositionSet() {}

    CommandMessagePositionSet(const Vector<> &position): MessageVector_Abstract(position) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_Position;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageVehicleModeSet: public MessageGeneric_Abstract<eVehicleMode_t> {
public:

    CommandMessageVehicleModeSet() {}

    CommandMessageVehicleModeSet(const eVehicleMode_t &vehicleMode): MessageGeneric_Abstract<eVehicleMode_t>(vehicleMode) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_VehicleMode;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageProgramStart: public KraftMessage_Interface {
public:

    CommandMessageProgramStart() {}

    CommandMessageProgramStart(const uint32_t& programID, const int64_t& programStartTime) {
        programID_ = programID;
        programStartTime_ = programStartTime;
    }

    void setProgramID(const uint32_t& programID) {programID_ = programID;} 
    const uint32_t& getProgramID() const {return programID_;}

    void setProgramStartTime(const uint32_t& programStartTime) {programStartTime_ = programStartTime;} 
    const int64_t& getProgramStartTime() const {return programStartTime_;}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_ProgramStart;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

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



class CommandMessageRCChannels: public KraftMessage_Interface {
public:

    CommandMessageRCChannels() {}

    /**
     * Constructor for array as input
     * @param channels Pointer to a int16_t array.
     * @param numChannels Number of channels to copy from channels.
     */
    CommandMessageRCChannels(int16_t* channels, const uint8_t &numChannels) {
        for (uint8_t i = 0; i < numChannels && i < c_maxChannels; i++) channels_[i] = channels[i];
    }

    virtual uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_RCChannels;}

    uint32_t getDataSize() const {return sizeof(channels_);}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

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



class CommandMessageMagCalValues: public MessageSensorCalibration_Abstract {
public:

    CommandMessageMagCalValues() {}

    CommandMessageMagCalValues(const FML::Vector3_F& bias, const FML::Matrix33_F& scaleAlign) : MessageSensorCalibration_Abstract(bias, scaleAlign) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_MagCalibData;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageAccelCalValues: public MessageSensorCalibration_Abstract {
public:

    CommandMessageAccelCalValues() {}

    CommandMessageAccelCalValues(const FML::Vector3_F& bias, const FML::Matrix33_F& scaleAlign) : MessageSensorCalibration_Abstract(bias, scaleAlign) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_AccelCalibData;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageGyroCalValues: public MessageSensorCalibration_Abstract {
public:

    CommandMessageGyroCalValues() {}

    CommandMessageGyroCalValues(const FML::Vector3_F& bias, const FML::Matrix33_F& scaleAlign) : MessageSensorCalibration_Abstract(bias, scaleAlign) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_GyroCalibData;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageMagMountTransform: public MessageMatrix33_Abstract {
public:

    CommandMessageMagMountTransform() {}

    CommandMessageMagMountTransform(const FML::Matrix33_F& matrix) : MessageMatrix33_Abstract(matrix) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_MagMountTransform;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageAccelMountTransform: public MessageMatrix33_Abstract {
public:

    CommandMessageAccelMountTransform() {}

    CommandMessageAccelMountTransform(const FML::Matrix33_F& matrix) : MessageMatrix33_Abstract(matrix) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_AccelMountTransform;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};



class CommandMessageGyroMountTransform: public MessageMatrix33_Abstract {
public:

    CommandMessageGyroMountTransform() {}

    CommandMessageGyroMountTransform(const FML::Matrix33_F& matrix) : MessageMatrix33_Abstract(matrix) {}

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_GyroMountTransform;}

    uint32_t getMessageType() const override {return eKraftMessageType_t::eKraftMessageType_Command_ID;}

};




#endif 
