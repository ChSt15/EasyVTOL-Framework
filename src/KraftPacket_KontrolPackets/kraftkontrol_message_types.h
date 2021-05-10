#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "lib/KraftKommunikation/src/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/vehicle_data.h"



enum eKraftMessageType_KraftKontrol_t : uint8_t {
    eKraftMessageType_KraftKontrol_Attitude = eKraftMessageType_t::eKraftMessageType_StandardEnd_ID, //Set first to ID of free not reserved IDs.
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_FullKinematics,
    eKraftMessageType_KraftKontrol_VehicleModeSet,
    eKraftMessageType_KraftKontrol_VehicleModeIs,
    eKraftMessageType_KraftKontrol_VehicleStatus,
    eKraftMessageType_KraftKontrol_RCChannels
};



class KraftMessageAttitude: public KraftMessage_Interface {
public:

    KraftMessageAttitude() {}

    KraftMessageAttitude(const Quaternion &attitude) {
        attitude_ = attitude;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_Attitude;}

    uint32_t getDataSize() {return sizeof(Quaternion);}

    Quaternion getAttitude() {return attitude_;}

    void setAttitude(Quaternion attitude) {attitude_ = attitude;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(Quaternion)) return false;

        memcpy(dataBytes, &attitude_, sizeof(Quaternion));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(Quaternion)) return false;

        memcpy(&attitude_, dataBytes, sizeof(Quaternion));

        return true;

    }


private:

    Quaternion attitude_;

};



class KraftMessagePosition: public KraftMessage_Interface {
public:

    KraftMessagePosition() {}

    KraftMessagePosition(const Vector &position, const uint32_t &timestamp) {
        position_ = position;
        timestamp_ = timestamp;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_Position;}

    uint32_t getDataSize() {return sizeof(Vector) + sizeof(timestamp_);}

    Vector getPosition() {return position_;}
    uint32_t getTimestamp() {return timestamp_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(Vector)) return false;

        memcpy(dataBytes, &position_, sizeof(Vector));
        memcpy(dataBytes + sizeof(Vector), &timestamp_, sizeof(uint32_t));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(Vector)) return false;

        memcpy(&position_, dataBytes, sizeof(Vector));
        memcpy(&timestamp_, dataBytes + sizeof(Vector), sizeof(uint32_t));

        return true;

    }


private:

    Vector position_;
    uint32_t timestamp_;

};


class KraftMessageFullKinematics: public KraftMessage_Interface {
public:

    KraftMessageFullKinematics() {}

    KraftMessageFullKinematics(const KinematicData &kinematics) {
        kinematics_ = kinematics;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_FullKinematics;}

    uint32_t getDataSize() {return sizeof(KinematicData);}

    KinematicData getKinematics() {return kinematics_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

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

    virtual uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeSet;}

    uint32_t getDataSize() {return sizeof(eVehicleMode_t);}

    eVehicleMode_t getVehicleMode() {return vehicleMode_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        memcpy(dataBytes, &vehicleMode_, sizeof(eVehicleMode_t));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        memcpy(&vehicleMode_, dataBytes, sizeof(eVehicleMode_t));

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

    virtual uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeIs;}

    uint32_t getDataSize() {return sizeof(eVehicleMode_t);}

    eVehicleMode_t getVehicleMode() {return vehicleMode_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        memcpy(dataBytes, &vehicleMode_, sizeof(eVehicleMode_t));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(eVehicleMode_t)) return false;

        memcpy(&vehicleMode_, dataBytes, sizeof(eVehicleMode_t));

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

    virtual uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleStatus;}

    uint32_t getDataSize() {return sizeof(VehicleData);}

    VehicleData getVehicleStatus() {return vehicleData_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(VehicleData)) return false;

        memcpy(dataBytes, &vehicleData_, sizeof(VehicleData));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(VehicleData)) return false;

        memcpy(&vehicleData_, dataBytes, sizeof(VehicleData));

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
        for (uint8_t i = 0; i < numChannels && i < sizeof(channels_)/sizeof(int16_t); i++) channels_[i] = channels[i];
    }

    virtual uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_RCChannels;}

    uint32_t getDataSize() {return sizeof(channels_);}

    float getChannel(const uint8_t &channel) {return channels_[constrain(channel, 0, 20)];}

    void getChannelAll(int16_t* channelArray, uint8_t numChannels = 15) {for (uint8_t i = 0; i < numChannels; i++) channelArray[i] = channels_[i];}

    void setChannel(const int16_t &value, const uint8_t &channel) {channels_[constrain(channel, 0, 20)] = value;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(channels_)) return false;

        memcpy(dataBytes, channels_, sizeof(channels_));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(channels_)) return false;

        memcpy(channels_, dataBytes, sizeof(channels_));

        return true;

    }


protected:

    int16_t channels_[15];

};


#endif 
