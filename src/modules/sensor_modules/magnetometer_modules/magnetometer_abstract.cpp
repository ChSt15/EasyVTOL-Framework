#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"


Magnetometer_Abstract::Magnetometer_Abstract() {

    axisScaleAlignMatrix_[0][0] = 1;
    axisScaleAlignMatrix_[1][1] = 1;
    axisScaleAlignMatrix_[2][2] = 1;

    mountTransformMatrix_ = axisScaleAlignMatrix_;

    axisBias_[0][0] = 0;
    axisBias_[1][0] = 0;
    axisBias_[2][0] = 0;

}


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Magnetometer_Abstract::getMagTopic() const {
    return valueTopic_;
}


void Magnetometer_Abstract::setMagCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    axisScaleAlignMatrix_ = axisScaleAlign;
}


const FML::Matrix33_F& Magnetometer_Abstract::getMagCalibrationAxisScaleAlign() const {
    return axisScaleAlignMatrix_;
}


void Magnetometer_Abstract::setMagCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    axisBias_ = axisBias;
}


const FML::Vector3_F& Magnetometer_Abstract::getMagCalibrationAxisBias() const {
    return axisBias_;
}


void Magnetometer_Abstract::setMagTransform(const FML::Matrix33_F& mountTransform) {
    mountTransformMatrix_ = mountTransform;
}


const FML::Matrix33_F& Magnetometer_Abstract::getMagTransform() const {
    return mountTransformMatrix_;
}


void Magnetometer_Abstract::publishMagData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> magValues, bool calibrate, bool transform) {

    if (calibrate) {
        magValues.data.values = axisScaleAlignMatrix_*(magValues.data.values + axisBias_);
        magValues.data.covariance = axisScaleAlignMatrix_*magValues.data.covariance;
    }

    if (transform) {
        magValues.data.values = mountTransformMatrix_*magValues.data.values;
        magValues.data.covariance = mountTransformMatrix_*magValues.data.covariance;
    }

    valueTopic_.publish(magValues);

}