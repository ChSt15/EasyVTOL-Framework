#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "lib/KraftKommunikation/src/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"



enum eKraftMessageType_KraftKontrol_t {
    eKraftMessageType_KraftKontrol_Attitude = eKraftMessageType_t::eKraftMessageType_StandardEnd_ID, //Set first to ID of free not reserved IDs.
    eKraftMessageType_KraftKontrol_Position,
    eKraftMessageType_KraftKontrol_FullKinematics
};



class KraftMessageAttitude final: public KraftMessage_Interface {
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



class KraftMessagePosition final: public KraftMessage_Interface {
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


class KraftMessageFullKinematics final: public KraftMessage_Interface {
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



#endif 
