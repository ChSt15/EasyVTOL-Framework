#ifndef KRAFTKONTROL_MESSAGETYPE_ABSTRACTS_H
#define KRAFTKONTROL_MESSAGETYPE_ABSTRACTS_H



#include "KraftKontrol/modules/communication_modules/kraft_message.h"

#include "lib/MathHelperLibrary/vector_math.h"
#include "lib/MathHelperLibrary/FML.h"

#include "KraftKontrol/data_containers/dynamic_data.h"
#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/vehicle_data.h"



class MessageQuaternion_Abstract: public KraftMessage_Interface {
public:

    void setQuaternion(const FML::Quaternion<>& quaternion) {quat_ = quaternion;}

    const FML::Quaternion<>& getQuaternion() const {return quat_;}


protected:

    MessageQuaternion_Abstract() {}

    MessageQuaternion_Abstract(const FML::Quaternion<> &quaternion) {
        quat_ = quaternion;
    }

    uint32_t getDataSize() const final {return sizeof(float)*4;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&quat_.w, sizeof(quat_.w));
        bufferWrite(&quat_.x, sizeof(quat_.x));
        bufferWrite(&quat_.y, sizeof(quat_.y));
        bufferWrite(&quat_.z, sizeof(quat_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override {

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&quat_.w, sizeof(quat_.w));
        bufferRead(&quat_.x, sizeof(quat_.x));
        bufferRead(&quat_.y, sizeof(quat_.y));
        bufferRead(&quat_.z, sizeof(quat_.z));
        endBufferRead();

        return true;

    }

    FML::Quaternion<> quat_;

};



class MessageVector_Abstract: public KraftMessage_Interface {
public:

    void setVector(const Vector<> &vector) {vector_ = vector;}

    const Vector<>& getVector() const {return vector_;}


protected:

    MessageVector_Abstract() {}

    MessageVector_Abstract(const Vector<> &vector) {
        vector_ = vector;
    }

    uint32_t getDataSize() const final {return sizeof(float)*3;}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&vector_.x, sizeof(vector_.x));
        bufferWrite(&vector_.y, sizeof(vector_.y));
        bufferWrite(&vector_.z, sizeof(vector_.z));
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override {

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&vector_.x, sizeof(vector_.x));
        bufferRead(&vector_.y, sizeof(vector_.y));
        bufferRead(&vector_.z, sizeof(vector_.z));
        endBufferRead();

        return true;

    }

    Vector<> vector_;

};



class MessageFullKinematics_Abstract: public KraftMessage_Interface {
public:

    void setKinematics(const KinematicData &kinematics) {kinematics_ = kinematics;}

    const KinematicData& getKinematics() const {return kinematics_;}


protected:

    MessageFullKinematics_Abstract() {}

    MessageFullKinematics_Abstract(const KinematicData &kinematics) {
        kinematics_ = kinematics;
    }

    uint32_t getDataSize() const {return sizeof(KinematicData);}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize < sizeof(KinematicData)) return false;

        memcpy(dataBytes, &kinematics_, sizeof(KinematicData));

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override{

        if (dataByteSize < sizeof(KinematicData)) return false;

        memcpy(&kinematics_, dataBytes, sizeof(KinematicData));

        return true;

    }

    KinematicData kinematics_;

};



class MessageFullDynamics_Abstract: public KraftMessage_Interface {
public:

    void setDynamics(const DynamicData &dynamics) {dynamics_ = dynamics;}

    const DynamicData& getDynamics() const {return dynamics_;}

protected:

    MessageFullDynamics_Abstract() {}

    MessageFullDynamics_Abstract(const DynamicData &dynamics) {
        dynamics_ = dynamics;
    }

    uint32_t getDataSize() const {return sizeof(DynamicData);}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize < sizeof(DynamicData)) return false;

        memcpy(dataBytes, &dynamics_, sizeof(DynamicData));

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override{

        if (dataByteSize < sizeof(DynamicData)) return false;

        memcpy(&dynamics_, dataBytes, sizeof(DynamicData));

        return true;

    }

    DynamicData dynamics_;

};



/**
 * Used to create custom message types easily. 
 * Should only be used for primitive datatypes! Otherwise problems between different platforms may occur!
 */
template <typename TYPE>
class MessageGeneric_Abstract: public KraftMessage_Interface {
public:

    void setData(const TYPE& data) {data_ = data;} 

    const TYPE& getData() const {return data_;}


protected:

    MessageGeneric_Abstract() {}

    MessageGeneric_Abstract(const TYPE& data) {
        data_ = data;
    }

    uint32_t getDataSize() const {return sizeof(TYPE);}

    bool getRawData(void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) const override {

        if (dataByteSize < getDataSize()) return false;

        startBufferWrite(dataBytes, startByte);
        bufferWrite(&data_, getDataSize());
        endBufferWrite();

        return true;

    }

    bool setRawData(const void* dataBytes, uint32_t dataByteSize, uint32_t startByte = 0) override{

        if (dataByteSize < getDataSize()) return false;

        startBufferRead(dataBytes, startByte);
        bufferRead(&data_, getDataSize());
        endBufferRead();

        return true;

    }

    TYPE data_;

};



#endif 
