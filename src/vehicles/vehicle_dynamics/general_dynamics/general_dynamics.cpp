#include "general_dynamics.h"



void GeneralDynamics::sensorFusionThread() {

    if (IMU::getDeviceStatus() != DeviceStatus::DEVICE_RUNNING) return;


    if (IMU::newData) {
        IMU::newData = false;

        float dt = 0.001f; //Timestep

        float beta = 0.001f; //Z-Axis correction factor
        float gamma = 0.1f; //X-Axis correction factor


        //Predict system state

        Vector rotationVector = IMU::lastGyro;

        if (rotationVector.magnitude() > 0.0f) {

            Quaternion rotationQuat = Quaternion(rotationVector, rotationVector.magnitude()*dt);

            _attitude = _attitude*rotationQuat;

        }


        //Correct state prediction

        //Z-Axis correction
        Vector zAxisIs = Vector(0,0,1);
        Vector zAxisSet = (_attitude*IMU::lastAccel.copy().normalize()*_attitude.copy().conjugate()).toVector();

        Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
        float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

        Quaternion zAxisCorrectionQuat(1,0,0,0);

        if (zAxisRotationAngle > 0.01*DEGREES) zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle*beta);

        
        //X-Axis correction
        Vector xAxisIs(1,0,0);
        Vector xAxisSet = (_attitude*IMU::lastMag.copy()*_attitude.copy().conjugate()).toVector();
        xAxisSet.z = 0;
        xAxisSet.normalize();

        Vector xAxisRotationAxis = Vector(0,0,1);
        float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

        Quaternion xAxisCorrectionQuat(1,0,0,0);

        if (xAxisRotationAngle > 0.01*DEGREES) xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle*gamma);


        //Apply state correction and normalise attitude quaternion 
        _attitude = zAxisCorrectionQuat*xAxisCorrectionQuat*_attitude;
        _attitude.normalize(true);

    }

}