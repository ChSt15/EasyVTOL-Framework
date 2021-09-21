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



class TelemetryMessageAttitude: public KraftMessageTelemetry_Abstract, public MessageQuaternion_Abstract {
public:

    TelemetryMessageAttitude() {}

    TelemetryMessageAttitude(const Quaternion<> &attitude): MessageQuaternion_Abstract(attitude) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Attitude;}

};



class TelemetryMessagePosition: public KraftMessageTelemetry_Abstract, public MessageVector_Abstract {
public:

    TelemetryMessagePosition() {}

    TelemetryMessagePosition(const Vector<> &position): MessageVector_Abstract(position) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Position;}

};



class TelemetryMessageFullKinematics: public KraftMessageTelemetry_Abstract, public MessageFullKinematics_Abstract {
public:

    TelemetryMessageFullKinematics() {}

    TelemetryMessageFullKinematics(const KinematicData &kinematics): MessageFullKinematics_Abstract(kinematics) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_FullKinematics;}

};



class TelemetryMessageForce: public KraftMessageTelemetry_Abstract, public MessageVector_Abstract {
public:

    TelemetryMessageForce() {}

    TelemetryMessageForce(const Vector<> &force): MessageVector_Abstract(force) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Force;}

};



class TelemetryMessageTorque: public KraftMessageTelemetry_Abstract, public MessageVector_Abstract {
public:

    TelemetryMessageTorque() {}

    TelemetryMessageTorque(const Vector<> &torque): MessageVector_Abstract(torque) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_Torque;}

};



class TelemetryMessageAngularVelocity: public MessageVector_Abstract {
public:

    TelemetryMessageAngularVelocity() {}

    TelemetryMessageAngularVelocity(const Vector<> &angularVelocity): MessageVector_Abstract(angularVelocity) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_AngularVelocity;}
    
    uint32_t getMessageType() const final override {return eKraftMessageType_t::eKraftMessageType_Telemetry_ID;}

};



class TelemetryMessageFullDynamics: public KraftMessageTelemetry_Abstract, public MessageFullDynamics_Abstract {
public:

    TelemetryMessageFullDynamics() {}

    TelemetryMessageFullDynamics(const DynamicData &dynamics): MessageFullDynamics_Abstract(dynamics) {}

    uint32_t getDataType() const final {return eMessageTypeTelemetry_t::eMessageTypeTelemetry_FullDynamics;}

};



#endif 
