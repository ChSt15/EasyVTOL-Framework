#ifndef KRAFTKONTROL_PACKETTYPES_H
#define KRAFTKONTROL_PACKETTYPES_H



#include "KraftPacketDatatype.h"

#include "3d_math.h"



enum KRAFTPACKET_KONTROL_DATA_ID {
    KRAFTPACKET_KONTROL_DATA_ATTITUDE_ID = KRAFTPACKET_DATA_FREE_ID, //Set first to ID of free not reserved IDs.
    KRAFTPACKET_KONTROL_DATA_RCCHANNELS_ID
};


class KraftDataAttitude final: public KraftDataType {
public:

    uint32_t getDataTypeID() {return KRAFTPACKET_KONTROL_DATA_ID::KRAFTPACKET_KONTROL_DATA_ATTITUDE_ID;}

    uint32_t getDataSize() {return sizeof(Quaternion);}

    Quaternion getAttitude() {return _attitude;}

    void setAttitude(const Quaternion &attitude) {
        
        _attitude = attitude;

    }

    bool getRawData(void* dataBytes, const uint32_t &dataByteSize) {

        if (dataByteSize != sizeof(Quaternion)) return false;

        memcpy(dataBytes, &_attitude, sizeof(Quaternion));

        return true;

    }

    bool setRawData(const void* dataBytes, const uint32_t &dataByteSize) {

        if (dataByteSize != sizeof(Quaternion)) return false;

        memcpy(&_attitude, dataBytes, sizeof(Quaternion));

        return true;

    }


private:

    Quaternion _attitude;

};



#endif 
