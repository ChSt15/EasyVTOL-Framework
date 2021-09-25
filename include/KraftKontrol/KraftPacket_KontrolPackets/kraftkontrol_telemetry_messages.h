#ifndef KRAFTKONTROL_TELEMETRY_MESSAGES_H
#define KRAFTKONTROL_TELEMETRY_MESSAGES_H



#include "kraftkontrol_data_messages.h"



enum eMessageTypeTelemetry_t: uint32_t {
    eMessageTypeTelemetry_AngularAcceleration,
    eMessageTypeTelemetry_AngularVelocity,
    eMessageTypeTelemetry_Attitude,
    eMessageTypeTelemetry_Acceleration,
    eMessageTypeTelemetry_Velocity,
    eMessageTypeTelemetry_Position,
    eMessageTypeTelemetry_FullKinematics,
    eMessageTypeTelemetry_Force,
    eMessageTypeTelemetry_Torque,
    eMessageTypeTelemetry_FullDynamics,
    eMessageTypeTelemetry_CurrentTime,
    eMessageTypeTelemetry_VehicleMode,
    eMessageTypeTelemetry_VehicleProgram,
    eMessageTypeTelemetry_GNSSData,
    eMessageTypeTelemetry_MagCalibData,
    eMessageTypeTelemetry_AccelCalibData
};



class TelemetryMessageAttitude: public MessageQuaternion_Abstract {
public:

    TelemetryMessageAttitude() {}

    TelemetryMessageAttitude(const Quaternion<> &attitude): MessageQuaternion_Abstract(attitude) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Attitude;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageVehicleModeIs: public MessageGeneric_Abstract<eVehicleMode_t> {
public:

    TelemetryMessageVehicleModeIs() {}

    TelemetryMessageVehicleModeIs(eVehicleMode_t vehicleModeIs): MessageGeneric_Abstract<eVehicleMode_t>(vehicleModeIs) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_VehicleMode;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessagePosition: public MessageVector_Abstract {
public:

    TelemetryMessagePosition() {}

    TelemetryMessagePosition(const Vector<> &position): MessageVector_Abstract(position) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Position;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageVelocity: public MessageVector_Abstract {
public:

    TelemetryMessageVelocity() {}

    TelemetryMessageVelocity(const Vector<> &velocity): MessageVector_Abstract(velocity) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Velocity;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageFullKinematics: public MessageFullKinematics_Abstract {
public:

    TelemetryMessageFullKinematics() {}

    TelemetryMessageFullKinematics(const KinematicData &kinematics): MessageFullKinematics_Abstract(kinematics) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_FullKinematics;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageForce: public MessageVector_Abstract {
public:

    TelemetryMessageForce() {}

    TelemetryMessageForce(const Vector<> &force): MessageVector_Abstract(force) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Force;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageTorque: public MessageVector_Abstract {
public:

    TelemetryMessageTorque() {}

    TelemetryMessageTorque(const Vector<> &torque): MessageVector_Abstract(torque) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Torque;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageAngularVelocity: public MessageVector_Abstract {
public:

    TelemetryMessageAngularVelocity() {}

    TelemetryMessageAngularVelocity(const Vector<> &angularVelocity): MessageVector_Abstract(angularVelocity) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_AngularVelocity;}
    
    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageFullDynamics: public MessageFullDynamics_Abstract {
public:

    TelemetryMessageFullDynamics() {}

    TelemetryMessageFullDynamics(const DynamicData &dynamics): MessageFullDynamics_Abstract(dynamics) {}

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_FullDynamics;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageGNSSData: public KraftMessage_Interface {
public:

    TelemetryMessageGNSSData() {}

    TelemetryMessageGNSSData(double latitude, double longitude, float height, uint8_t numSats) {
        lon_ = longitude;
        lat_ = latitude;
        height_ = height;
        numSats_ = numSats;
    }

    uint32_t getDataType() const final override {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_GNSSData;}

    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

    uint32_t getDataSize() const {return sizeof(lon_) + sizeof(lat_) + sizeof(numSats_) + sizeof(height_);}

    double getLatitude() {return lat_;}
    double getLongitude() {return lon_;}
    float getHeight() {return height_;}
    uint8_t getNumSats() {return numSats_;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize != getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite((uint8_t*)&lon_, sizeof(lon_));
        bufferWrite((uint8_t*)&lat_, sizeof(lat_));
        bufferWrite((uint8_t*)&height_, sizeof(height_));
        bufferWrite((uint8_t*)&numSats_, sizeof(numSats_));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override{

        if (dataByteSize != getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead((uint8_t*)&lon_, sizeof(lon_));
        bufferRead((uint8_t*)&lat_, sizeof(lat_));
        bufferRead((uint8_t*)&height_, sizeof(height_));
        bufferRead((uint8_t*)&numSats_, sizeof(numSats_));
        endBufferRead();

        return true;

    }

private:

    double lon_, lat_;
    float height_;
    uint8_t numSats_ = 0;

};



#endif 
