#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"


Accelerometer_Abstract::Accelerometer_Abstract() {

    axisScaleAlignMatrix_[0][0] = 1;
    axisScaleAlignMatrix_[1][1] = 1;
    axisScaleAlignMatrix_[2][2] = 1;

    mountTransformMatrix_ = axisScaleAlignMatrix_;

    axisBias_[0][0] = 0;
    axisBias_[1][0] = 0;
    axisBias_[2][0] = 0;

}


const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& Accelerometer_Abstract::getAccelTopic() const {
    return valueTopic_;
}


void Accelerometer_Abstract::setAccelCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign) {
    axisScaleAlignMatrix_ = axisScaleAlign;
}


const FML::Matrix33_F& Accelerometer_Abstract::getAccelCalibrationAxisScaleAlign() const {
    return axisScaleAlignMatrix_;
}


void Accelerometer_Abstract::setAccelCalibrationAxisBias(const FML::Vector3_F& axisBias) {
    axisBias_ = axisBias;
}


const FML::Vector3_F& Accelerometer_Abstract::getAccelCalibrationAxisBias() const {
    return axisBias_;
}


void Accelerometer_Abstract::setAccelTransform(const FML::Matrix33_F& mountTransform) {
    mountTransformMatrix_ = mountTransform;
}


const FML::Matrix33_F& Accelerometer_Abstract::getAccelTransform() const {
    return mountTransformMatrix_;
}


void Accelerometer_Abstract::publishAccelData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> accelValues, bool calibrate, bool transform) {

    if (calibrate) {
        accelValues.data.values = axisScaleAlignMatrix_*(accelValues.data.values + axisBias_);
        accelValues.data.covariance = axisScaleAlignMatrix_*accelValues.data.covariance;
    }

    if (transform) {
        accelValues.data.values = mountTransformMatrix_*accelValues.data.values;
        accelValues.data.covariance = mountTransformMatrix_*accelValues.data.covariance;
    }

    valueTopic_.publish(accelValues);

}