#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"


Magnetometer_Abstract::Magnetometer_Abstract() {

    FML::Matrix33_F axisScaleAlignMatrix_;
    FML::Matrix33_F mountTransformMatrix_;
    FML::Vector3_F axisBias_;

    axisScaleAlignMatrix_[0][0] = 1;
    axisScaleAlignMatrix_[1][1] = 1;
    axisScaleAlignMatrix_[2][2] = 1;

    mountTransformMatrix_ = axisScaleAlignMatrix_;

    axisBias_[0][0] = 0;
    axisBias_[1][0] = 0;
    axisBias_[2][0] = 0;

    calValues_.setBias(axisBias_);
    calValues_.setScaleAlign(axisScaleAlignMatrix_);
    mountValues_.setMatrix(mountTransformMatrix_);

    messageSubr_.addReceiverMessage(calValues_);
    messageSubr_.addReceiverMessage(mountValues_);

    messageSubr_.subscribe(Module_Abstract::getGlobalMessageTopic());

}


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Magnetometer_Abstract::getMagTopic() const {
    return valueTopic_;
}


void Magnetometer_Abstract::setMagCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    calValues_.setScaleAlign(axisScaleAlign);
}


const FML::Matrix33_F& Magnetometer_Abstract::getMagCalibrationAxisScaleAlign() const {
    return calValues_.getScaleAlign();
}


void Magnetometer_Abstract::setMagCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    calValues_.setBias(axisBias);
}


const FML::Vector3_F& Magnetometer_Abstract::getMagCalibrationAxisBias() const {
    return calValues_.getBias();
}


void Magnetometer_Abstract::setMagTransform(const FML::Matrix33_F& mountTransform) {
    mountValues_.setMatrix(mountTransform);
}


const FML::Matrix33_F& Magnetometer_Abstract::getMagTransform() const {
    return mountValues_.getMatrix();
}


void Magnetometer_Abstract::publishMagData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> magValues, bool calibrate, bool transform) {

    //Serial.print(  String("Bias: x: ") + String(calValues_.getBias()(0),4) + ", y: " + String(calValues_.getBias()(1),4) + ", z: " + String(calValues_.getBias()(2),4));
    //Serial.println(String(" Scale: x: ") + String(calValues_.getScaleAlign()(0,0),4) + ", y: " + String(calValues_.getScaleAlign()(1,1),4) + ", z: " + String(calValues_.getScaleAlign()(2,2),4));

    if (calibrate) {
        magValues.data.values = calValues_.getScaleAlign()*(magValues.data.values - calValues_.getBias());
        magValues.data.covariance = calValues_.getScaleAlign()*magValues.data.covariance;
    }

    if (transform) {
        magValues.data.values = mountValues_.getMatrix()*magValues.data.values;
        magValues.data.covariance = mountValues_.getMatrix()*magValues.data.covariance;
    }

    valueTopic_.publish(magValues);

}