#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"


Accelerometer_Abstract::Accelerometer_Abstract() {

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


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Accelerometer_Abstract::getAccelTopic() const {
    return valueTopic_;
}


void Accelerometer_Abstract::setAccelCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    calValues_.setScaleAlign(axisScaleAlign);
}


const FML::Matrix33_F& Accelerometer_Abstract::getAccelCalibrationAxisScaleAlign() const {
    return calValues_.getScaleAlign();
}


void Accelerometer_Abstract::setAccelCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    calValues_.setBias(axisBias);
}


const FML::Vector3_F& Accelerometer_Abstract::getAccelCalibrationAxisBias() const {
    return calValues_.getBias();
}


void Accelerometer_Abstract::setAccelTransform(const FML::Matrix33_F& mountTransform) {
    mountValues_.setMatrix(mountTransform);
}


const FML::Matrix33_F& Accelerometer_Abstract::getAccelTransform() const {
    return mountValues_.getMatrix();
}


void Accelerometer_Abstract::publishAccelData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> accelValues, bool calibrate, bool transform) {

    if (calibrate) {
        accelValues.data.values = calValues_.getScaleAlign()*(accelValues.data.values - calValues_.getBias());
        accelValues.data.covariance = calValues_.getScaleAlign()*accelValues.data.covariance;
    }

    if (transform) {
        accelValues.data.values = mountValues_.getMatrix()*accelValues.data.values;
        accelValues.data.covariance = mountValues_.getMatrix()*accelValues.data.covariance;
    }

    valueTopic_.publish(accelValues);

}