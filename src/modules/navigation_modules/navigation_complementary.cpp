#include "navigation_complementary.h"



void NavigationComplementaryFilter::thread() {

    if (gyro_ == nullptr || accel_ == nullptr) {

        stopTaskThreading();

    }

    //Calculate time delta from last run
    float dTime = (float)(micros() - _lastLoopTimestamp)/1000000.0f;
    _lastLoopTimestamp = micros();
    	
    //Predict current state
    //NavigationData prediction = navigationData_;
    navigationData_.velocity = navigationData_.velocity + navigationData_.linearAcceleration*dTime;
    navigationData_.position.x += navigationData_.velocity.x*dTime;
    navigationData_.position.y += navigationData_.velocity.y*dTime;

    navigationData_.absolutePosition.height = navigationData_.absolutePosition.height + navigationData_.velocity.z*dTime;

    navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;


    //Correct with sensor values
    while (gyro_->gyroAvailable() > 0) {
        
        //Get IMU data
        Vector rotationVector;
        uint32_t timestamp;
        gyro_->getGyro(&rotationVector, &timestamp);

        if (rotationVector.magnitude() < 0.1) {
            gyroLPF_.update(rotationVector);
        }
        rotationVector = rotationVector - gyroLPF_.getValue();

        //Calulate time delta
        float dt = float(timestamp - _lastGyroTimestamp)/1000000.0f;
        _lastGyroTimestamp = timestamp;

        //Calulate derivitive of gyro for angular acceleration
        navigationData_.angularAcceleration = (rotationVector - _lastGyroValue)/dt;

        //Check if gyro initialised
        if (_gyroInitialized) {

            //rotationVector = rotationVector - _gyroHPF.update(rotationVector, timestamp);

            //Predict system state
            if (!rotationVector.isZeroVector()) {

                Quaternion rotationQuat = Quaternion(rotationVector, rotationVector.magnitude()*dt);

                navigationData_.attitude = navigationData_.attitude*rotationQuat;

            }

            //Update angularRate
            navigationData_.angularRate = (navigationData_.attitude*rotationVector*navigationData_.attitude.copy().conjugate()).toVector(); //Transform angular rate into world coordinate system

        } else {

            //Gyro filter initialisation
            if (rotationVector.magnitude() < 0.05) {
                gyroLPF_.setValue(rotationVector);
                _gyroInitialized = true;
            }

        }

    }


    while (accel_->accelAvailable() > 0) {

        //static Vector lastValue = 0;

        //Get IMU data
        Vector accelVector;
        uint32_t timestamp;
        accel_->getAccel(&accelVector, &timestamp);

        accelVector = (accelVector - _accelBias).compWiseMulti(_accelScale);

        accelVector = accelLPF_.update(accelVector);

        //accelVector = lastValue = lastValue*0.9999 + accelVector*0.0001;

        //Serial.println("Accel: x: " + String(accelVector.x,4) + ", y: " + String(accelVector.y,4) + ", z: " + String(accelVector.z,4));

        //Check if accelerometer initialised
        if (_accelInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - _lastAccelTimestamp)/1000000.0f;
            _lastAccelTimestamp = timestamp;

            float beta = 0.1f;

            //Z-Axis correction
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle*beta*dt);


            //Apply state correction and normalise attitude quaternion 
            navigationData_.attitude = zAxisCorrectionQuat*navigationData_.attitude;
            navigationData_.attitude.normalize(true);


            //Update acceleration
            navigationData_.acceleration = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector(); //Transform acceleration into world coordinate system and remove gravity
            Vector filtered = accelBiasLPF_.update(navigationData_.acceleration - Vector(0,0,9.81));
            navigationData_.linearAcceleration = navigationData_.acceleration - Vector(0,0,9.81) - filtered;//accelHPF_.update(navigationData_.acceleration/* - Vector(0,0,9.81)*/);

            //Serial.println(String("Accel: x: ") + navigationData_.linearAcceleration.x + ", y: " + navigationData_.linearAcceleration.y + ", z: " + navigationData_.linearAcceleration.z);
            
            

        } else if (_gyroInitialized) {
            
            //Accel filter initialisation
            _accelInitialized = true;
            _lastAccelTimestamp = timestamp;

            //Set Attitude
            Vector zAxisIs = Vector(0,0,1);
            Vector zAxisSet = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector();

            Vector zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion zAxisCorrectionQuat = Quaternion(zAxisRotationAxis, zAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            navigationData_.attitude = zAxisCorrectionQuat*navigationData_.attitude;
            navigationData_.attitude.normalize(true);
            //navigationData_.attitude = Quaternion(0,0,1,0);

            accelBiasLPF_.setValue(navigationData_.acceleration - Vector(0,0,9.81));

        }

    }


    if (mag_ != nullptr) {

        while (mag_->magAvailable() > 0) {

            /*static Vector max = -1000;
            static Vector min = 1000;
            static Vector offset = 0;
            static Vector scale = 1;*/

            //Get IMU data
            Vector magVector;
            uint32_t timestamp;
            mag_->getMag(&magVector, &timestamp);

            if (_magInitialized) {

                /*if (magVector.x > max.x) {
                    max.x = magVector.x;
                }
                if (magVector.x < min.x) {
                    min.x = magVector.x;
                }
                if (magVector.y > max.y) {
                    max.y = magVector.y;
                }
                if (magVector.y < min.y) {
                    min.y = magVector.y;
                }
                if (magVector.z > max.z) {
                    max.z = magVector.z;
                }
                if (magVector.z < min.z) {
                    min.z = magVector.z;
                }

                offset = (min + max)/2;
                float avg = (max.x-offset.x + max.y-offset.y + max.z-offset.z)/3;
                scale.x = avg/(max.x-offset.x);
                scale.y = avg/(max.y-offset.y);
                scale.z = avg/(max.z-offset.z);

                Serial.println(String("Max: x:") + max.x + ", y:" + max.y + ", z:" + max.z);
                Serial.println(String("Min: x:") + min.x + ", y:" + min.y + ", z:" + min.z);
                Serial.println(String("Max-offset: x:") + (max.x - offset.x) + ", y:" + (max.y - offset.y) + ", z:" + (max.z - offset.z));
                Serial.println(String("Offset: x:") + offset.x + ", y:" + offset.y + ", z:" + offset.z);
                Serial.println(String("Scale: x:") + scale.x + ", y:" + scale.y + ", z:" + scale.z + ", avg: " + avg);
                Serial.println();*/


                magVector = (magVector - _magOffset).compWiseMulti(_magScale);

                //Correct state prediction
                float dt = float(timestamp - _lastMagTimestamp)/1000000.0f;
                _lastMagTimestamp = timestamp;

                float gamma = 0.1f;

                //X-Axis correction
                Vector xAxisIs(1,0,0);
                Vector xAxisSet = (navigationData_.attitude*magVector*navigationData_.attitude.copy().conjugate()).toVector();
                xAxisSet.z = 0;
                xAxisSet.normalize();

                Vector xAxisRotationAxis = Vector(0,0,1);
                float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

                Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle*gamma*dt);


                //Apply state correction and normalise attitude quaternion 
                navigationData_.attitude = xAxisCorrectionQuat*navigationData_.attitude;
                navigationData_.attitude.normalize(true);

            } else if (_accelInitialized) {

                //Magnetometer filter initialisation
                _magInitialized = true;
                _lastMagTimestamp = timestamp;

                //Set heading
                Vector xAxisIs(1,0,0);
                Vector xAxisSet = (navigationData_.attitude*magVector*navigationData_.attitude.copy().conjugate()).toVector();
                xAxisSet.z = 0;
                xAxisSet.normalize();

                Vector xAxisRotationAxis = Vector(0,0,1);
                float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

                Quaternion xAxisCorrectionQuat = Quaternion(xAxisRotationAxis, xAxisRotationAngle);

                //Apply state correction and normalise attitude quaternion 
                navigationData_.attitude = xAxisCorrectionQuat*navigationData_.attitude;
                navigationData_.attitude.normalize(true);

            }

        }

    }


    //Make sure baro module is valid before using.
    if (baro_ != nullptr) {

        while (baro_->pressureAvailable() > 0) {

            //Get IMU data
            float pressure;
            uint32_t timestamp;
            baro_->getPressure(&pressure, &timestamp);

            //Check if accelerometer initialised
            if (_baroInitialized) {
                
                //Correct state prediction
                float dt = float(timestamp - _lastBaroTimestamp)/1000000.0f;
                _lastBaroTimestamp = timestamp;

                float beta = 0.01f;

                //calculate height from new pressure value
                float heightAbsolute = _getHeightFromPressure(pressure, 100e3f);
                //float heightRelative = heightAbsolute - navigationData_.absolutePosition.height;
                //calculate z velocity from new height value
                float zVelocity = (heightAbsolute - _lastHeightValue)/dt;

                //correct dead reckoning values with new ones.
                float heightError = (heightAbsolute - navigationData_.absolutePosition.height);
                navigationData_.absolutePosition.height += heightError*0.03;
                navigationData_.velocity.z += (zVelocity - navigationData_.velocity.z)*0.02;

                //Update relative position
                navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;

                //Update last Height value
                _lastHeightValue = heightAbsolute;


            } else {
                
                //Set flag to true
                _baroInitialized = true;
                
                //Barometer filter initialisation
                _lastBaroTimestamp = timestamp;
                _lastHeightValue = _getHeightFromPressure(pressure, 100e3f);

                //Set current calculated height as start value.
                navigationData_.absolutePosition.height = _lastHeightValue;

            }

        }

    }

    
    
    if (gnss_ != nullptr) {

        while (gnss_->positionAvailable() > 0) {

            WorldPosition positionAbsolute; uint32_t time;
            if (gnss_->getPosition(&positionAbsolute, &time)) {

                float beta = 0.1;

                navigationData_.absolutePosition.latitude = positionAbsolute.latitude;
                navigationData_.absolutePosition.longitude = positionAbsolute.longitude;

                Vector positionBuf = positionAbsolute.getPositionVectorFrom(navigationData_.homePosition);

                navigationData_.position.x += (positionBuf.x - navigationData_.position.x)*beta;
                navigationData_.position.y += (positionBuf.y - navigationData_.position.y)*beta;

            }

            Vector velocityBuf;
            if (gnss_->getVelocity(&velocityBuf, &time)) {

                float beta = 0.1;

                navigationData_.velocity += (velocityBuf - navigationData_.velocity)*beta;

            }

        }

    }


    navigationData_.timestamp = micros();

}
