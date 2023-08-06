#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"


Gyroscope_Abstract::Gyroscope_Abstract() {

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


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Gyroscope_Abstract::getGyroTopic() const {
    return valueTopic_;
}


void Gyroscope_Abstract::setGyroCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    calValues_.setScaleAlign(axisScaleAlign);
}


const FML::Matrix33_F& Gyroscope_Abstract::getGyroCalibrationAxisScaleAlign() const {
    return calValues_.getScaleAlign();
}


void Gyroscope_Abstract::setGyroCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    calValues_.setBias(axisBias);
}


const FML::Vector3_F& Gyroscope_Abstract::getGyroCalibrationAxisBias() const {
    return calValues_.getBias();
}


void Gyroscope_Abstract::setGyroTransform(const FML::Matrix33_F& mountTransform) {
    mountValues_.setMatrix(mountTransform);
}


const FML::Matrix33_F& Gyroscope_Abstract::getGyroTransform() const {
    return mountValues_.getMatrix();
}


void Gyroscope_Abstract::publishGyroData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> gyroValues, bool calibrate, bool transform) {
    
    if (calibrate) {
        gyroValues.data.values = calValues_.getScaleAlign()*(gyroValues.data.values - calValues_.getBias());
        gyroValues.data.covariance = calValues_.getScaleAlign()*gyroValues.data.covariance;
    }

    if (transform) {
        gyroValues.data.values = mountValues_.getMatrix()*gyroValues.data.values;
        gyroValues.data.covariance = mountValues_.getMatrix()*gyroValues.data.covariance;
    }

    valueTopic_.publish(gyroValues);

}