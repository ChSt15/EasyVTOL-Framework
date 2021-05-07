#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "lib/KraftKommunikation/src/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/vehicle_data.h"



enum eKraftMessageType_KraftKontrol_t {
    eKraftMessageType_KraftKontrol_Attitude = eKraftMessageType_t::eKraftMessageType_StandardEnd_ID, //Set first to ID of free not reserved IDs.
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_FullKinematics,
    eKraftMessageType_KraftKontrol_VehicleModeSet,
    eKraftMessageType_KraftKontrol_VehicleModeIs,
    eKraftMessageType_KraftKontrol_VehicleStatus
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
class KraftMessageVehicleModeIs: KraftMessageVehicleModeSet {
public:

    KraftMessageVehicleModeIs() {}

    KraftMessageVehicleModeIs(const eVehicleMode_t &vehicleMode) {
        vehicleMode_ = vehicleMode;
    }

    uint32_t getDataTypeID() override {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeIs;}

};


class KraftMessageVehicleStatus: public KraftMessage_Interface {
public:

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


#endif 
