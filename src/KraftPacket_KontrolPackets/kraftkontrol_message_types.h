#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "lib/KraftKommunikation/src/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/vehicle_data.h"



enum eKraftMessageType_KraftKontrol_t : uint8_t {
    eKraftMessageType_KraftKontrol_AttitudeSet = eKraftMessageType_t::eKraftMessageType_StandardEnd_ID, //Set first to ID of free not reserved IDs.
    eKraftMessageType_KraftKontrol_AttitudeIs,
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_FullKinematics,
    eKraftMessageType_KraftKontrol_VehicleModeSet,
    eKraftMessageType_KraftKontrol_VehicleModeIs,
    eKraftMessageType_KraftKontrol_VehicleStatus,
    eKraftMessageType_KraftKontrol_RCChannels,
    eKraftMessageType_KraftKontrol_GNSSData,
    eKraftMessageType_KraftKontrol_MagCalibIs,
    eKraftMessageType_KraftKontrol_MagCalibSet
};



class KraftMessageAttitudeIs: public KraftMessage_Interface {
public:

    KraftMessageAttitudeIs() {}

    KraftMessageAttitudeIs(const Quaternion<> &attitude) {
        attitude_ = attitude;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_AttitudeIs;}

    uint32_t getDataSize() const {return sizeof(Quaternion<>);}

    Quaternion<> getAttitude() const {return attitude_;}

    void setAttitude(Quaternion<> attitude) {attitude_ = attitude;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(Quaternion<>)) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&attitude_.w, sizeof(attitude_.w));
        bufferWrite(&attitude_.x, sizeof(attitude_.x));
        bufferWrite(&attitude_.y, sizeof(attitude_.y));
        bufferWrite(&attitude_.z, sizeof(attitude_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(Quaternion<>)) return false;

        startBufferRead(dataBytes);
        bufferRead(&attitude_.w, sizeof(attitude_.w));
        bufferRead(&attitude_.x, sizeof(attitude_.x));
        bufferRead(&attitude_.y, sizeof(attitude_.y));
        bufferRead(&attitude_.z, sizeof(attitude_.z));
        endBufferRead();

        return true;

    }


private:

    Quaternion<> attitude_;

};



class KraftMessageAttitudeSet: public KraftMessage_Interface {
public:

    KraftMessageAttitudeSet() {}

    KraftMessageAttitudeSet(const Quaternion<> &attitude) {
        attitude_ = attitude;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_AttitudeSet;}

    uint32_t getDataSize() const {return sizeof(Quaternion<>);}

    Quaternion<> getAttitude() {return attitude_;}

    void setAttitude(Quaternion<> attitude) {attitude_ = attitude;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(Quaternion<>)) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&attitude_.w, sizeof(attitude_.w));
        bufferWrite(&attitude_.x, sizeof(attitude_.x));
        bufferWrite(&attitude_.y, sizeof(attitude_.y));
        bufferWrite(&attitude_.z, sizeof(attitude_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(Quaternion<>)) return false;

        startBufferRead(dataBytes);
        bufferRead(&attitude_.w, sizeof(attitude_.w));
        bufferRead(&attitude_.x, sizeof(attitude_.x));
        bufferRead(&attitude_.y, sizeof(attitude_.y));
        bufferRead(&attitude_.z, sizeof(attitude_.z));
        endBufferRead();

        return true;

    }


private:

    Quaternion<> attitude_;

};



class KraftMessagePosition: public KraftMessage_Interface {
public:

    KraftMessagePosition() {}

    KraftMessagePosition(const Vector<> &position) {
        position_ = position;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_Position;}

    uint32_t getDataSize() const {return sizeof(Vector<>);}

    Vector<> getPosition() {return position_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&position_.x, sizeof(position_.x));
        bufferWrite(&position_.y, sizeof(position_.y));
        bufferWrite(&position_.z, sizeof(position_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes);
        bufferRead(&position_.x, sizeof(position_.x));
        bufferRead(&position_.y, sizeof(position_.y));
        bufferRead(&position_.z, sizeof(position_.z));
        endBufferRead();

        return true;

    }


private:

    Vector<> position_;

};


class KraftMessageFullKinematics: public KraftMessage_Interface {
public:

    KraftMessageFullKinematics() {}

    KraftMessageFullKinematics(const KinematicData &kinematics) {
        kinematics_ = kinematics;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_FullKinematics;}

    uint32_t getDataSize() const {return sizeof(KinematicData);}

    KinematicData getKinematics() {return kinematics_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(KinematicData)) return false;

        memcpy(dataBytes, &kinematics_, sizeof(KinematicData));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(KinematicData)) return false;

        memcpy(&kinematics_, dataBytes, sizeof(KinematicData));

        return true;

    }


private:

    KinematicData kinematics_;

};



class KraftMessageVehicleModeSet: public KraftMessage_Interface {
public:

    KraftMessageVehicleModeSet() {}

    KraftMessageVehicleModeSet(const eVehicleMode_t &vehicleMode) {
        vehicleMode_ = vehicleMode;
    }

    virtual uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeSet;}

    uint32_t getDataSize() const {return sizeof(eVehicleMode_t);}

    eVehicleMode_t getVehicleMode() {return vehicleMode_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&vehicleMode_, sizeof(vehicleMode_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        startBufferRead(dataBytes);
        bufferRead(&vehicleMode_, sizeof(vehicleMode_));
        endBufferRead();

        return true;

    }


protected:

    eVehicleMode_t vehicleMode_;

};


//Inheret from other class to simplify stuff
class KraftMessageVehicleModeIs: public KraftMessage_Interface {
public:

    KraftMessageVehicleModeIs() {}

    KraftMessageVehicleModeIs(const eVehicleMode_t &vehicleMode) {
        vehicleMode_ = vehicleMode;
    }

    virtual uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeIs;}

    uint32_t getDataSize() const {return sizeof(eVehicleMode_t);}

    eVehicleMode_t getVehicleMode() {return vehicleMode_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&vehicleMode_, sizeof(vehicleMode_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        startBufferRead(dataBytes);
        bufferRead(&vehicleMode_, sizeof(vehicleMode_));
        endBufferRead();

        return true;

    }


protected:

    eVehicleMode_t vehicleMode_;


};


class KraftMessageVehicleStatus: public KraftMessage_Interface {
public:

    KraftMessageVehicleStatus() {}

    KraftMessageVehicleStatus(const VehicleData &vehicleData) {
        vehicleData_ = vehicleData;
    }

    virtual uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleStatus;}

    uint32_t getDataSize() const {return sizeof(VehicleData);}

    VehicleData getVehicleStatus() {return vehicleData_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(VehicleData)) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&vehicleData_.vehicleMode, sizeof(vehicleData_.vehicleMode));
        bufferWrite(&vehicleData_.vehicleReady, sizeof(vehicleData_.vehicleReady));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(VehicleData)) return false;

        startBufferRead(dataBytes);
        bufferRead(&vehicleData_.vehicleMode, sizeof(vehicleData_.vehicleMode));
        bufferRead(&vehicleData_.vehicleReady, sizeof(vehicleData_.vehicleReady));
        endBufferRead();

        return true;

    }


protected:

    VehicleData vehicleData_;

};



class KraftMessageRCChannels: public KraftMessage_Interface {
public:

    KraftMessageRCChannels() {}

    /**
     * Constructor for array as input
     * @param channels Pointer to a int16_t array.
     * @param numChannels Number of channels to copy from channels.
     */
    KraftMessageRCChannels(int16_t* channels, const uint8_t &numChannels) {
        for (uint8_t i = 0; i < numChannels && i < c_maxChannels; i++) channels_[i] = channels[i];
    }

    virtual uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_RCChannels;}

    uint32_t getDataSize() const {return sizeof(channels_);}

    int16_t getChannel(const uint8_t &channel) {return channels_[channel];}

    void getChannelAll(int16_t* channelArray, uint8_t numChannels = c_maxChannels) {for (uint8_t i = 0; i < numChannels; i++) channelArray[i] = channels_[i];}

    void setChannel(const int16_t &value, const uint8_t &channel) {channels_[channel] = value;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < sizeof(channels_)) return false;

        startBufferWrite(dataBytes);
        for (uint8_t i = 0; i < c_maxChannels; i++) bufferWrite(channels_, sizeof(channels_[0]));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(channels_)) return false;

        startBufferRead(dataBytes);
        for (uint8_t i = 0; i < c_maxChannels; i++) bufferRead(channels_, sizeof(channels_[0]));
        endBufferRead();

        return true;

    }


protected:

    static const uint8_t c_maxChannels = 15;

    int16_t channels_[c_maxChannels];

};



class KraftMessageGNSSData: public KraftMessage_Interface {
public:

    KraftMessageGNSSData() {}

    /**
     * @param position Position data.
     * @param sats Number of sats.
     */
    KraftMessageGNSSData(WorldPosition position, uint8_t sats) {
        position_ = position;
        sats_ = sats;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_GNSSData;}

    uint32_t getDataSize() const {return sizeof(position_) + sizeof(sats_);}

    WorldPosition getPosition() {return position_;}

    uint8_t getNumSats() {return sats_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes);
        bufferWrite(&position_.latitude, sizeof(position_.latitude));
        bufferWrite(&position_.longitude, sizeof(position_.longitude));
        bufferWrite(&position_.height, sizeof(position_.height));
        bufferWrite(&sats_, sizeof(sats_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes);
        bufferRead(&position_.latitude, sizeof(position_.latitude));
        bufferRead(&position_.longitude, sizeof(position_.longitude));
        bufferRead(&position_.height, sizeof(position_.height));
        bufferRead(&sats_, sizeof(sats_));
        endBufferWrite();

        return true;

    }


protected:

    WorldPosition position_;
    uint8_t sats_ = 0;

};



class KraftMessageMagCalValuesIs: public KraftMessage_Interface {
public:

    KraftMessageMagCalValuesIs() {}

    KraftMessageMagCalValuesIs(const Vector<> &magMax, const Vector<> &magMin) {
        magMin_ = magMin;
        magMax_ = magMax;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_MagCalibIs;}

    uint32_t getDataSize() const {return sizeof(magMin_) + sizeof(magMax_);}

    Vector<> getMinValue() const {return magMin_;}
    Vector<> getMaxValue() const {return magMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {magMin_ = magMin; magMax_ = magMax;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes);
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

        startBufferRead(dataBytes);
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



class KraftMessageMagCalValuesSet: public KraftMessage_Interface {
public:

    KraftMessageMagCalValuesSet() {}

    KraftMessageMagCalValuesSet(const Vector<> &magMax, const Vector<> &magMin) {
        magMin_ = magMin;
        magMax_ = magMax;
    }

    uint32_t getDataTypeID() const {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_MagCalibSet;}

    uint32_t getDataSize() const {return sizeof(magMin_) + sizeof(magMax_);}

    Vector<> getMinValue() const {return magMin_;}
    Vector<> getMaxValue() const {return magMax_;}

    void setMinMax(const Vector<> &magMax, const Vector<> &magMin) {magMin_ = magMin; magMax_ = magMax;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) const {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes);
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

        startBufferRead(dataBytes);
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



#endif 
