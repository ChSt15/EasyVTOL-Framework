#include "navigation.h"



void Navigation::navigationThread() {

    if (!_interval.isTimeToRun()) return;


    if (IMU::getDeviceStatus() != DeviceStatus::DEVICE_RUNNING) return;


    if (IMU::gyroAvailable()) {
        
        //Get IMU data
        Vector rotationVector;
        uint32_t timestamp;
        IMU::getGyro(&rotationVector, &timestamp);

        //Check if gyro initialised
        if (gyroInitialized) {

            //Predict system state
            float dt = float(timestamp - _lastGyroTimestamp)/1000000.0f;
            _lastGyroTimestamp = timestamp;

            if (!rotationVector.isZeroVector()) {

                Quaternion rotationQuat = Quaternion(rotationVector, rotationVector.magnitude()*dt);

                _kineticData.attitude = _kineticData.attitude*rotationQuat;

            }

            //Update angularRate
            _kineticData.angularRate = (_kineticData.attitude*rotationVector*_kineticData.attitude.copy().conjugate()).toVector(); //Transform angular rate into world coordinate system

        } else {

            //Gyro filter initialisation
            gyroInitialized = true;
            _lastGyroTimestamp = timestamp;

        }

    }


    if (IMU::accelAvailable()) {

        //Get IMU data
        Vector accelVector;
        uint32_t timestamp;
        IMU::getAccel(&accelVector, &timestamp);

        //Check if accelerometer initialised
        if (accelInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - _lastAccelTimestamp)/1000000.0f;
            _lastAccelTimestamp = timestamp;

            float beta = 1.0f;

            //Z-Axis correction
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (_kineticData.attitude*accelVector*_kineticData.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle*beta*dt);


            //Apply state correction and normalise attitude quaternion 
            _kineticData.attitude = zAxisCorrectionQuat*_kineticData.attitude;
            _kineticData.attitude.normalize(true);


            //Update acceleration
            _kineticData.acceleration = (_kineticData.attitude*accelVector*_kineticData.attitude.copy().conjugate()).toVector(); //Transform acceleration into world coordinate system and remove gravity
            _kineticData.linearAcceleration = _kineticData.acceleration - Vector(0,0,9.81);

        } else if (gyroInitialized) {
            
            //Accel filter initialisation
            accelInitialized = true;
            _lastAccelTimestamp = timestamp;

            //Set Attitude
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (_kineticData.attitude*accelVector*_kineticData.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            _kineticData.attitude = zAxisCorrectionQuat*_kineticData.attitude;
            _kineticData.attitude.normalize(true);

        }

    }


    if (IMU::magAvailable()) {

        //Get IMU data
        Vector magVector;
        uint32_t timestamp;
        IMU::getMag(&magVector, &timestamp);

        if (magInitialized) {

            //Correct state prediction
            float dt = float(timestamp - _lastMagTimestamp)/1000000.0f;
            _lastMagTimestamp = timestamp;

            float gamma = 0.1f;

            //X-Axis correction
            Vector xAxisIs(1,0,0);
            Vector xAxisSet = (_kineticData.attitude*magVector*_kineticData.attitude.copy().conjugate()).toVector();
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector xAxisRotationAxis = Vector(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle*gamma*dt);


            //Apply state correction and normalise attitude quaternion 
            _kineticData.attitude = xAxisCorrectionQuat*_kineticData.attitude;
            _kineticData.attitude.normalize(true);

        } else if (accelInitialized) {

            //Magnetometer filter initialisation
            magInitialized = true;
            _lastMagTimestamp = timestamp;

            //Set heading
            Vector xAxisIs(1,0,0);
            Vector xAxisSet = (_kineticData.attitude*magVector*_kineticData.attitude.copy().conjugate()).toVector();
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector xAxisRotationAxis = Vector(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            _kineticData.attitude = xAxisCorrectionQuat*_kineticData.attitude;
            _kineticData.attitude.normalize(true);

        }

    }


    //################## Inertial navigation testing ###############

    float dt = 1.0f/LOOP_RATE_LIMIT;

    _kineticData.velocity = _kineticData.velocity + _kineticData.acceleration*dt;

    _kineticData.position = _kineticData.position + _kineticData.velocity*dt;

}



void Navigation::navigationInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer) {

    _flightMode = flightModePointer;
    _flightProfile = flightProfilePointer;
    
}