#include "navigation_complementary.h"



void NavigationComplementary::thread() {

    if (IMU::getDeviceStatus() != DeviceStatus::DEVICE_RUNNING) return;

    float dTime = (float)(micros() - _lastLoopTimestamp)/1000000.0f;
    _lastLoopTimestamp = micros();
    	
    //KinematicData *_navigationData.= _vehicle;

    if (IMU::gyroAvailable()) {
        
        //Get IMU data
        Vector rotationVector;
        uint32_t timestamp;
        IMU::getGyro(&rotationVector, &timestamp);

        //Calulate time delta
        float dt = float(timestamp - _lastGyroTimestamp)/1000000.0f;
        _lastGyroTimestamp = timestamp;

        //Calulate derivitive of gyro for angular acceleration
        _navigationData.angularAcceleration = (rotationVector - _lastGyroValue)/dt;

        //Check if gyro initialised
        if (_gyroInitialized) {

            //Predict system state
            if (!rotationVector.isZeroVector()) {

                Quaternion rotationQuat = Quaternion(rotationVector, rotationVector.magnitude()*dt);

                _navigationData.attitude = _navigationData.attitude*rotationQuat;

            }

            //Update angularRate
            _navigationData.angularRate = (_navigationData.attitude*rotationVector*_navigationData.attitude.copy().conjugate()).toVector(); //Transform angular rate into world coordinate system

        } else {

            //Gyro filter initialisation
            _gyroInitialized = true;

        }

    }


    if (IMU::accelAvailable()) {

        //Get IMU data
        Vector accelVector;
        uint32_t timestamp;
        IMU::getAccel(&accelVector, &timestamp);

        //Check if accelerometer initialised
        if (_accelInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - _lastAccelTimestamp)/1000000.0f;
            _lastAccelTimestamp = timestamp;

            float beta = 1.0f;

            //Z-Axis correction
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (_navigationData.attitude*accelVector*_navigationData.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle*beta*dt);


            //Apply state correction and normalise attitude quaternion 
            _navigationData.attitude = zAxisCorrectionQuat*_navigationData.attitude;
            _navigationData.attitude.normalize(true);


            //Update acceleration
            _navigationData.acceleration = (_navigationData.attitude*accelVector*_navigationData.attitude.copy().conjugate()).toVector(); //Transform acceleration into world coordinate system and remove gravity
            _navigationData.linearAcceleration = _navigationData.acceleration - Vector(0,0,9.81);

        } else if (_gyroInitialized) {
            
            //Accel filter initialisation
            _accelInitialized = true;
            _lastAccelTimestamp = timestamp;

            //Set Attitude
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (_navigationData.attitude*accelVector*_navigationData.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            _navigationData.attitude = zAxisCorrectionQuat*_navigationData.attitude;
            _navigationData.attitude.normalize(true);

        }

    }


    if (IMU::magAvailable()) {

        //Get IMU data
        Vector magVector;
        uint32_t timestamp;
        IMU::getMag(&magVector, &timestamp);

        if (_magInitialized) {

            //Correct state prediction
            float dt = float(timestamp - _lastMagTimestamp)/1000000.0f;
            _lastMagTimestamp = timestamp;

            float gamma = 0.1f;

            //X-Axis correction
            Vector xAxisIs(1,0,0);
            Vector xAxisSet = (_navigationData.attitude*magVector*_navigationData.attitude.copy().conjugate()).toVector();
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector xAxisRotationAxis = Vector(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle*gamma*dt);


            //Apply state correction and normalise attitude quaternion 
            _navigationData.attitude = xAxisCorrectionQuat*_navigationData.attitude;
            _navigationData.attitude.normalize(true);

        } else if (_accelInitialized) {

            //Magnetometer filter initialisation
            _magInitialized = true;
            _lastMagTimestamp = timestamp;

            //Set heading
            Vector xAxisIs(1,0,0);
            Vector xAxisSet = (_navigationData.attitude*magVector*_navigationData.attitude.copy().conjugate()).toVector();
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector xAxisRotationAxis = Vector(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            _navigationData.attitude = xAxisCorrectionQuat*_navigationData.attitude;
            _navigationData.attitude.normalize(true);

        }

    }


    //################## Inertial navigation testing ###############

    _navigationData.velocity = _navigationData.velocity + _navigationData.acceleration*dTime;

    _navigationData.position = _navigationData.position + _navigationData.velocity*dTime;

}



void NavigationComplementary::init() {
    
}