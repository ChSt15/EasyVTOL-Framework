#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "lib/KraftKommunikation/src/kraft_message.h"

#include "lib/Math-Helper/src/3d_math.h"



enum eKraftMessageType_KraftKontrol_t {
    eKraftMessageType_KraftKontrol_Attitude = eKraftMessageType_t::eKraftMessageType_StandardEnd_ID, //Set first to ID of free not reserved IDs.
    eKraftMessageType_KraftKontrol_Position
};


class KraftDataAttitude final: public KraftMessage_Interface {
public:

    KraftDataAttitude() {}

    KraftDataAttitude(const Quaternion &attitude) {
        attitude_ = attitude;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_Attitude;}

    uint32_t getDataSize() {return sizeof(Quaternion);}

    Quaternion getAttitude() {return attitude_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(Quaternion)) return false;

        memcpy(dataBytes, (void*)&attitude_, sizeof(Quaternion));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(Quaternion)) return false;

        memcpy((void*)&attitude_, &dataBytes, sizeof(Quaternion));

        return true;

    }


private:

    Quaternion attitude_;

};



class KraftDataPosition final: public KraftMessage_Interface {
public:

    KraftDataPosition() {}

    KraftDataPosition(const Vector &position, const uint32_t &timestamp) {
        position_ = position;
        timestamp_ = timestamp;
    }

    uint32_t getDataTypeID() {return eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_Position;}

    uint32_t getDataSize() {return sizeof(Vector) + sizeof(timestamp_);}

    Vector getPositiion() {return position_;}
    uint32_t getTimestamp() {return timestamp_;}

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0) {

        if (dataByteSize < sizeof(Vector)) return false;

        memcpy(dataBytes, (void*)&position_, sizeof(Vector));
        memcpy(dataBytes + sizeof(Vector), (void*)&timestamp_, sizeof(uint32_t));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize, const uint32_t &startByte = 0){

        if (dataByteSize < sizeof(Vector)) return false;

        memcpy((void*)&position_, &dataBytes, sizeof(Vector));
        memcpy((void*)&timestamp_, &dataBytes + sizeof(Vector), sizeof(uint32_t));

        return true;

    }


private:

    Vector position_;
    uint32_t timestamp_;

};



#endif 
