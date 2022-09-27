#include "KraftKontrol/modules/sensor_modules/imu_modules/imu_simulator.h"

#include "KraftKontrol/utils/random.h"



void IMUSimulator::thread() {


    DataTimestamped<FML::Quat_F> attitude;
    if (attitudeSubr_.isDataNew()) attitude = attitudeSubr_.getItem();

    DataTimestamped<FML::Matrix_F<3, 1>> position;
    if (positionSubr_.isDataNew()) position = positionSubr_.getItem();


    if (attitude.timestamp != 0 && position.timestamp != 0) {//Both have been updated

        //Attitude stuff
        auto deltaAtt = attitude.data * lastAttitude_.data.conjugate();

        FML::Matrix_F<3, 1> rotVec(deltaAtt.toRotVec() / (float(attitude.timestamp - lastAttitude_.timestamp) / SECONDS));

        SensorData<FML::Vector3_F, FML::Matrix33_F> dataOut;
        dataOut.values = rotVec;
        dataOut.values(0) += randNorm(gyroVariance_);
        dataOut.values(1) += randNorm(gyroVariance_);
        dataOut.values(2) += randNorm(gyroVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(gyroVariance_);
        publishGyroData(dataOut, false, false);

        FML::Vector3_F magVec; //Roughly 55 deg pointing downwards
        magVec(0) = 28;
        magVec(1) = 0;
        magVec(2) = -41;

        dataOut.values = attitude.data.conjugate().rotateVec(magVec);
        dataOut.values(0) += randNorm(magVariance_);
        dataOut.values(1) += randNorm(magVariance_);
        dataOut.values(2) += randNorm(magVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(magVariance_);
        publishMagData(dataOut, false, false);

        //Position stuff
        float dt = float(position.timestamp - lastPosition_.timestamp) / SECONDS;

        auto velocity = (position.data - lastPosition_.data) / dt;
        auto accel = (velocity - lastVelocity_) / dt;
        accel(2) += 9.8066f; //Gravity

        lastAccel_ = accel;
        accel = attitude.data.rotateVec(accel);

        SensorData<FML::Vector3_F, FML::Matrix33_F> dataOut;
        dataOut.values = accel;
        dataOut.values(0) += randNorm(accelVariance_);
        dataOut.values(1) += randNorm(accelVariance_);
        dataOut.values(2) += randNorm(accelVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(accelVariance_);
        publishAccelData(dataOut, false, false);

        lastVelocity_ = velocity;
        lastPosition_ = position;
        lastAttitude_ = attitude;

    } else if (attitude.timestamp != 0) { //Only attitude updated

        auto deltaAtt = attitude.data * lastAttitude_.data.conjugate();

        FML::Matrix_F<3, 1> rotVec(deltaAtt.toRotVec() / (float(attitude.timestamp - lastAttitude_.timestamp) / SECONDS));

        SensorData<FML::Vector3_F, FML::Matrix33_F> dataOut;
        dataOut.values = rotVec;
        dataOut.values(0) += randNorm(gyroVariance_);
        dataOut.values(1) += randNorm(gyroVariance_);
        dataOut.values(2) += randNorm(gyroVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(gyroVariance_);
        publishGyroData(dataOut, false, false);

        FML::Vector3_F magVec; //Roughly 55 deg pointing downwards
        magVec(0) = 28;
        magVec(1) = 0;
        magVec(2) = -41;

        dataOut.values = attitude.data.conjugate().rotateVec(magVec);
        dataOut.values(0) += randNorm(magVariance_);
        dataOut.values(1) += randNorm(magVariance_);
        dataOut.values(2) += randNorm(magVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(magVariance_);
        publishMagData(dataOut, false, false);

        //Update accel with new attitude. Rotation will change accel in sensor frame
        auto accel = attitude.data.rotateVec(lastAccel_);

        SensorData<FML::Vector3_F, FML::Matrix33_F> dataOut;
        dataOut.values = accel;
        dataOut.values(0) += randNorm(accelVariance_);
        dataOut.values(1) += randNorm(accelVariance_);
        dataOut.values(2) += randNorm(accelVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(accelVariance_);
        publishAccelData(dataOut, false, false);

        lastAttitude_ = attitude;

    } else if (position.timestamp != 0) {

        float dt = float(position.timestamp - lastPosition_.timestamp) / SECONDS;

        auto velocity = (position.data - lastPosition_.data) / dt;
        auto accel = (velocity - lastVelocity_) / dt;
        accel(2) += GRAVITY_MAGNITUDE; //Gravity

        lastAccel_ = accel;
        accel = lastAttitude_.data.rotateVec(accel);

        SensorData<FML::Vector3_F, FML::Matrix33_F> dataOut;
        dataOut.values = accel;
        dataOut.values(0) += randNorm(accelVariance_);
        dataOut.values(1) += randNorm(accelVariance_);
        dataOut.values(2) += randNorm(accelVariance_);
        dataOut.covariance = FML::Matrix33_F::eye(accelVariance_);
        publishAccelData(dataOut, false, false);

        lastVelocity_ = velocity;
        lastPosition_ = position;

    }
    
}


void IMUSimulator::init() {

    moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    attitudeSubr_.setTaskToResume(*this);
    positionSubr_.setTaskToResume(*this);

}
