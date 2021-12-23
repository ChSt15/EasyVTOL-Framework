#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"


Gyroscope_Abstract::Gyroscope_Abstract() {

    axisScaleAlignMatrix_[0][0] = 1;
    axisScaleAlignMatrix_[1][1] = 1;
    axisScaleAlignMatrix_[2][2] = 1;

    mountTransformMatrix_ = axisScaleAlignMatrix_;

    axisBias_[0][0] = 0;
    axisBias_[1][0] = 0;
    axisBias_[2][0] = 0;

}


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Gyroscope_Abstract::getGyroTopic() const {
    return valueTopic_;
}


void Gyroscope_Abstract::setGyroCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    axisScaleAlignMatrix_ = axisScaleAlign;
}


const FML::Matrix33_F& Gyroscope_Abstract::getGyroCalibrationAxisScaleAlign() const {
    return axisScaleAlignMatrix_;
}


void Gyroscope_Abstract::setGyroCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    axisBias_ = axisBias;
}


const FML::Vector3_F& Gyroscope_Abstract::getGyroCalibrationAxisBias() const {
    return axisBias_;
}


void Gyroscope_Abstract::setGyroTransform(const FML::Matrix33_F& mountTransform) {
    mountTransformMatrix_ = mountTransform;
}


const FML::Matrix33_F& Gyroscope_Abstract::getGyroTransform() const {
    return mountTransformMatrix_;
}


void Gyroscope_Abstract::publishGyroData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> gyroValues, bool calibrate, bool transform) {

    if (calibrate) {
        gyroValues.data.values = axisScaleAlignMatrix_*(gyroValues.data.values + axisBias_);
        gyroValues.data.covariance = axisScaleAlignMatrix_*gyroValues.data.covariance;
    }

    if (transform) {
        gyroValues.data.values = mountTransformMatrix_*gyroValues.data.values;
        gyroValues.data.covariance = mountTransformMatrix_*gyroValues.data.covariance;
    }

    valueTopic_.publish(gyroValues);

}