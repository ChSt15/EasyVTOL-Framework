#include "navigation_complementary.h"



void NavigationComplementaryFilter::thread() {

    if (gyro_ == nullptr || accel_ == nullptr) {

        stopTaskThreading();

    }

    //if (navigationData_.absolutePosition)

    //Calculate time delta from last run
    float dTime = (float)(micros() - _lastLoopTimestamp)/1000000.0f;
    _lastLoopTimestamp = micros();
    	
    //Predict current state
    velocityDeadReckoning_ += accelDeadReckoning_*dTime;
    positionDeadReckoning_ += velocityDeadReckoning_*dTime;


    //Correct with sensor values
    while (gyro_->gyroAvailable() > 0) {
        
        //Get IMU data
        Vector<> rotationVector;
        uint32_t timestamp;
        gyro_->getGyro(&rotationVector, &timestamp);

        if (rotationVector.magnitude() < 0.1) {
            gyroLPF_.update(rotationVector);
        }
        rotationVector = rotationVector - gyroLPF_.getValue();

        gyroXBuffer_.placeFront(rotationVector.x, true);
        gyroYBuffer_.placeFront(rotationVector.y, true);
        gyroZBuffer_.placeFront(rotationVector.z, true);
        rotationVector = Vector<>(gyroXBuffer_.getMedian(), gyroYBuffer_.getMedian(), gyroZBuffer_.getMedian());

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

                Quaternion<> rotationQuat(rotationVector, rotationVector.magnitude()*dt);

                navigationData_.attitude = navigationData_.attitude*rotationQuat;

            }

            //Update angularRate
            navigationData_.angularRate = (navigationData_.attitude*rotationVector*navigationData_.attitude.copy().conjugate()).toVector(); //Transform angular rate into world coordinate system

        } else {

            //Gyro filter initialisation
            if (rotationVector.magnitude() < 0.01) {
                gyroLPF_.setValue(rotationVector);
                _gyroInitialized = true;
            }

        }

    }


    while (accel_->accelAvailable() > 0) {

        //static Vector lastValue = 0;

        //Get IMU data
        Vector<> accelVector;
        uint32_t timestamp;
        accel_->getAccel(&accelVector, &timestamp);

        accelVector = (accelVector - _accelBias).compWiseMulti(_accelScale);

        accelVector = accelLPF_.update(accelVector);

        accelXBuffer_.placeFront(accelVector.x, true);
        accelYBuffer_.placeFront(accelVector.y, true);
        accelZBuffer_.placeFront(accelVector.z, true);
        accelVector = Vector<>(accelXBuffer_.getMedian(), accelYBuffer_.getMedian(), accelZBuffer_.getMedian());

        //accelVector = lastValue = lastValue*0.9999 + accelVector*0.0001;

        //Serial.println("Accel: x: " + String(accelVector.x,4) + ", y: " + String(accelVector.y,4) + ", z: " + String(accelVector.z,4));

        //Check if accelerometer initialised
        if (_accelInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - _lastAccelTimestamp)/1000000.0f;
            _lastAccelTimestamp = timestamp;

            float beta = 0.1f;

            //Z-Axis correction
            Vector<> zAxisIs = Vector<>(0,0,1);
            Vector<> zAxisSet = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector();

            Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion<> zAxisCorrectionQuat = Quaternion<>(zAxisRotationAxis, zAxisRotationAngle*beta*dt);


            //Apply state correction and normalise attitude quaternion 
            navigationData_.attitude = zAxisCorrectionQuat*navigationData_.attitude;
            navigationData_.attitude.normalize(true);


            //Update acceleration
            navigationData_.acceleration = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector(); //Transform acceleration into world coordinate system and remove gravity
            Vector<> filtered = accelBiasLPF_.update(navigationData_.acceleration - Vector<>(0,0,9.81));
            navigationData_.linearAcceleration = navigationData_.acceleration - Vector<>(0,0,9.81) - filtered;//accelHPF_.update(navigationData_.acceleration/* - Vector<>(0,0,9.81)*/);

            //Update linear acceleration
            ValueError<> buf = ValueError<>(navigationData_.linearAcceleration.x, accelXBuffer_.getStandardError());
            accelDeadReckoning_.value.x = buf.value;
            accelDeadReckoning_.error.x = buf.error;

            buf = ValueError<>(navigationData_.linearAcceleration.y, accelXBuffer_.getStandardError());
            accelDeadReckoning_.value.y = buf.value;
            accelDeadReckoning_.error.y = buf.error;

            buf = ValueError<>(navigationData_.linearAcceleration.z, accelXBuffer_.getStandardError());
            accelDeadReckoning_.value.z = buf.value;
            accelDeadReckoning_.error.z = buf.error;

            //Serial.println(String("Accel: x: ") + navigationData_.linearAcceleration.x + ", y: " + navigationData_.linearAcceleration.y + ", z: " + navigationData_.linearAcceleration.z);
            
            

        } else if (_gyroInitialized) {
            
            //Accel filter initialisation
            _accelInitialized = true;
            _lastAccelTimestamp = timestamp;

            //Set Attitude
            Vector<> zAxisIs = Vector<>(0,0,1);
            Vector<> zAxisSet = (navigationData_.attitude*accelVector*navigationData_.attitude.copy().conjugate()).toVector();

            Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion<> zAxisCorrectionQuat = Quaternion<>(zAxisRotationAxis, zAxisRotationAngle);

            //Apply state correction and normalise attitude quaternion 
            navigationData_.attitude = zAxisCorrectionQuat*navigationData_.attitude;
            navigationData_.attitude.normalize(true);
            //navigationData_.attitude = Quaternion(0,0,1,0);

            accelBiasLPF_.setValue(navigationData_.acceleration - Vector<>(0,0,9.81));

        }

    }


    if (mag_ != nullptr) {

        while (mag_->magAvailable() > 0) {

            /*static Vector max = -1000;
            static Vector min = 1000;
            static Vector offset = 0;
            static Vector scale = 1;*/

            //Get IMU data
            Vector<> magVector;
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
                Vector<> xAxisIs(1,0,0);
                Vector<> xAxisSet = (navigationData_.attitude*magVector*navigationData_.attitude.copy().conjugate()).toVector();
                xAxisSet.z = 0;
                xAxisSet.normalize();

                Vector<> xAxisRotationAxis = Vector<>(0,0,1);
                float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

                Quaternion<> xAxisCorrectionQuat = Quaternion<>(xAxisRotationAxis, xAxisRotationAngle*gamma*dt);


                //Apply state correction and normalise attitude quaternion 
                navigationData_.attitude = xAxisCorrectionQuat*navigationData_.attitude;
                navigationData_.attitude.normalize(true);

            } else if (_accelInitialized) {

                //Magnetometer filter initialisation
                _magInitialized = true;
                _lastMagTimestamp = timestamp;

                //Set heading
                Vector<> xAxisIs(1,0,0);
                Vector<> xAxisSet = (navigationData_.attitude*magVector*navigationData_.attitude.copy().conjugate()).toVector();
                xAxisSet.z = 0;
                xAxisSet.normalize();

                Vector<> xAxisRotationAxis = Vector<>(0,0,1);
                float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

                Quaternion<> xAxisCorrectionQuat = Quaternion<>(xAxisRotationAxis, xAxisRotationAngle);

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
                _lastHeightValue = heightAbsolute;

                //Update buffers
                baroHeightBuffer_.placeFront(heightAbsolute, true);
                baroVelBuffer_.placeFront(zVelocity, true);

                float heightMedian = baroHeightBuffer_.getMedian();
                float heightError = baroHeightBuffer_.getStandardError();

                float velMedian = baroVelBuffer_.getMedian();
                float velError = baroVelBuffer_.getStandardError();

                //correct dead reckoning values with new ones.
                ValueError<> heightVel = ValueError<>(velocityDeadReckoning_.value.z, velocityDeadReckoning_.error.z).weightedAverage(ValueError<>(velMedian, velError));
                velocityDeadReckoning_.value.z = heightVel.value;
                velocityDeadReckoning_.error.z = heightVel.error;

                ValueError<> height = ValueError<>(positionDeadReckoning_.value.z, positionDeadReckoning_.error.z).weightedAverage(ValueError<>(heightMedian, heightError));
                positionDeadReckoning_.value.z = height.value;
                positionDeadReckoning_.error.z = height.error;

                //Update output values
                navigationData_.velocity.z = velocityDeadReckoning_.value.z;
                navigationData_.absolutePosition.height = positionDeadReckoning_.value.z;
                navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;


            } else {
                
                //Set flag to true
                _baroInitialized = true;
                
                //Barometer filter initialisation
                _lastBaroTimestamp = timestamp;
                _lastHeightValue = _getHeightFromPressure(pressure, 100e3f);

                baroHeightBuffer_.placeFront(_lastHeightValue, true);
                baroVelBuffer_.placeFront(0, true);

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

                Vector<> positionBuf = positionAbsolute.getPositionVectorFrom(navigationData_.homePosition);

                gnssPositionXBuffer_.placeFront(positionBuf.x, true);
                gnssPositionYBuffer_.placeFront(positionBuf.y, true);
                gnssPositionZBuffer_.placeFront(positionBuf.z, true);

                if (gnssPositionXBuffer_.available() >= 2) {

                    ValueError<Vector<>> position;
                    position.value = Vector<>(gnssPositionXBuffer_.getMedian(), gnssPositionYBuffer_.getMedian(), gnssPositionZBuffer_.getMedian());
                    position.error = Vector<>(gnssPositionXBuffer_.getStandardError(), gnssPositionYBuffer_.getStandardError(), gnssPositionZBuffer_.getStandardError());

                    position = positionDeadReckoning_.weightedAverage(position);

                    positionDeadReckoning_.value.x = position.value.x;
                    positionDeadReckoning_.error.x = position.error.x;
                    positionDeadReckoning_.value.y = position.value.y;
                    positionDeadReckoning_.error.y = position.error.y;

                }

            }

            Vector<> velocityBuf;
            if (gnss_->getVelocity(&velocityBuf, &time)) {

                gnssVelocityXBuffer_.placeFront(velocityBuf.x, true);
                gnssVelocityYBuffer_.placeFront(velocityBuf.y, true);
                gnssVelocityZBuffer_.placeFront(velocityBuf.z, true);

                if (gnssVelocityXBuffer_.available() >= 2) {

                    ValueError<Vector<>> velocity;
                    velocity.value = Vector<>(gnssVelocityXBuffer_.getMedian(), gnssVelocityYBuffer_.getMedian(), gnssVelocityZBuffer_.getMedian());
                    velocity.error = Vector<>(gnssVelocityXBuffer_.getStandardError(), gnssVelocityYBuffer_.getStandardError(), gnssVelocityZBuffer_.getStandardError());

                    velocityDeadReckoning_ = velocityDeadReckoning_.weightedAverage(velocity);

                }

            }

        }

    }



    //Update output parameters
    navigationData_.velocity = velocityDeadReckoning_.value;
    navigationData_.velocityError = velocityDeadReckoning_.error;

    navigationData_.position = positionDeadReckoning_.value;
    navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;

    navigationData_.absolutePosition.height = positionDeadReckoning_.value.z;

    navigationData_.positionError = positionDeadReckoning_.error;

    navigationData_.timestamp = micros();

}
