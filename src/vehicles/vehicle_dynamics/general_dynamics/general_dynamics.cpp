#include "general_dynamics.h"



void GeneralDynamics::sensorFusionThread() {

    if (IMU::getDeviceStatus() != DeviceStatus::DEVICE_RUNNING) return;


    if (IMU::gyroFifo.available()) {

        //Predict system state

        float dt = 0.001f; //Timestep

        Vector rotationVector = IMU::gyroFifo.pop();

        if (rotationVector.magnitude() > 0.0f) {

            Quaternion rotationQuat = Quaternion(rotationVector, rotationVector.magnitude()*dt);

            _attitude = _attitude*rotationQuat;

        }

    }


    if (IMU::accelFifo.available()) {

        //Correct state prediction

        float beta = 0.001f;

        //Z-Axis correction
        Vector zAxisIs = Vector(0,0,1);
        Vector zAxisSet = (_attitude*IMU::accelFifo.pop().normalize()*_attitude.copy().conjugate()).toVector();

        Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
        float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

        Quaternion zAxisCorrectionQuat(1,0,0,0);

        if (zAxisRotationAngle > 0.01*DEGREES) zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle*beta);

        _attitude = zAxisCorrectionQuat*_attitude;
        _attitude.normalize(true);

    }
    

    if (IMU::magFifo.available()) {

        //Correct state prediction

        float gamma = 0.1f;

        //X-Axis correction
        Vector xAxisIs(1,0,0);
        Vector xAxisSet = (_attitude*IMU::magFifo.pop().normalize()*_attitude.copy().conjugate()).toVector();
        xAxisSet.z = 0;
        xAxisSet.normalize();

        Vector xAxisRotationAxis = Vector(0,0,1);
        float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

        Quaternion xAxisCorrectionQuat(1,0,0,0);

        if (xAxisRotationAngle > 0.01*DEGREES) xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle*gamma);


        //Apply state correction and normalise attitude quaternion 
        _attitude = xAxisCorrectionQuat*_attitude;
        _attitude.normalize(true);

    }

}