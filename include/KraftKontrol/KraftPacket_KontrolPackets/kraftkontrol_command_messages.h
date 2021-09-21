#ifndef KRAFTKONTROL_COMMAND_MESSAGES_H
#define KRAFTKONTROL_COMMAND_MESSAGES_H



#include "KraftKontrol/modules/communication_modules/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/vehicle_data.h"



enum eMessageTypeCommand_t : uint32_t {
    eKraftMessageType_KraftKontrol_Attitude,
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_VehicleMode,
    eKraftMessageType_KraftKontrol_RCChannels,
    eKraftMessageType_KraftKontrol_MagCalib,
    eKraftMessageType_KraftKontrol_AccelCalib,
    eKraftMessageType_KraftKontrol_ProgramStart
};



class CommandMessageAttitudeSet: public KraftMessageCommand_Abstract, public MessageQuaternion_Abstract {
public:

    CommandMessageAttitudeSet() {}

    CommandMessageAttitudeSet(const Quaternion<> &attitude): MessageQuaternion_Abstract(attitude) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_Attitude;}

};



class CommandMessagePositionSet: public KraftMessageCommand_Abstract, public MessageVector_Abstract {
public:

    CommandMessagePositionSet() {}

    CommandMessagePositionSet(const Vector<> &position): MessageVector_Abstract(position) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_Position;}

};



class CommandMessageVehicleModeSet: public KraftMessageCommand_Abstract, public MessageGeneric_Abstract<eVehicleMode_t> {
public:

    CommandMessageVehicleModeSet() {}

    CommandMessageVehicleModeSet(const eVehicleMode_t &vehicleMode): MessageGeneric_Abstract<eVehicleMode_t>(vehicleMode) {}

    uint32_t getDataType() const final {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_VehicleMode;}

};



class CommandMessageProgramStart: public KraftMessageCommand_Abstract {
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


    uint32_t getDataSize() const {return sizeof(programID_) + sizeof(programStartTime_);}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&programID_, sizeof(programID_));
        bufferWrite(&programStartTime_, sizeof(programStartTime_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

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



class CommandMessageRCChannels: public KraftMessageCommand_Abstract {
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

    int16_t getChannel(const uint8_t &channel) {return channels_[channel];}

    void getChannelAll(int16_t* channelArray, uint8_t numChannels = c_maxChannels) {for (uint8_t i = 0; i < numChannels; i++) channelArray[i] = channels_[i];}

    void setChannel(const int16_t &value, const uint8_t &channel) {channels_[channel] = value;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        for (uint8_t i = 0; i < c_maxChannels; i++) bufferWrite(channels_, sizeof(channels_[0]));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

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



class CommandMessageMagCalValues: public KraftMessageCommand_Abstract {
public:

    CommandMessageMagCalValues() {}

    CommandMessageMagCalValues(const Vector<> &magMax, const Vector<> &magMin) {
        magMin_ = magMin;
        magMax_ = magMax;
    }

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_MagCalib;}

    uint32_t getDataSize() const {return sizeof(magMin_) + sizeof(magMax_);}

    Vector<> getMinValue() const {return magMin_;}
    Vector<> getMaxValue() const {return magMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {magMin_ = magMin; magMax_ = magMax;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

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

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

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


private:

    Vector<> magMin_ = 90000000;
    Vector<> magMax_ = -90000000;

};



class CommandMessageAccelCalValues: public KraftMessageCommand_Abstract {
public:

    CommandMessageAccelCalValues() {}

    CommandMessageAccelCalValues(const Vector<> &accelMax, const Vector<> &accelMin) {
        accelMin_ = accelMin;
        accelMax_ = accelMax;
    }

    uint32_t getDataType() const {return eMessageTypeCommand_t::eKraftMessageType_KraftKontrol_MagCalib;}

    uint32_t getDataSize() const {return sizeof(accelMin_) + sizeof(accelMax_);}

    Vector<> getMinValue() const {return accelMin_;}
    Vector<> getMaxValue() const {return accelMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {accelMin_ = magMin; accelMax_ = magMax;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&accelMin_.x, sizeof(accelMin_.x));
        bufferWrite(&accelMin_.y, sizeof(accelMin_.y));
        bufferWrite(&accelMin_.z, sizeof(accelMin_.z));
        bufferWrite(&accelMax_.x, sizeof(accelMax_.x));
        bufferWrite(&accelMax_.y, sizeof(accelMax_.y));
        bufferWrite(&accelMax_.z, sizeof(accelMax_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&accelMin_.x, sizeof(accelMin_.x));
        bufferRead(&accelMin_.y, sizeof(accelMin_.y));
        bufferRead(&accelMin_.z, sizeof(accelMin_.z));
        bufferRead(&accelMax_.x, sizeof(accelMax_.x));
        bufferRead(&accelMax_.y, sizeof(accelMax_.y));
        bufferRead(&accelMax_.z, sizeof(accelMax_.z));
        endBufferRead();

        return true;

    }


private:

    Vector<> accelMin_ = 90000000;
    Vector<> accelMax_ = -90000000;

};



#endif 
