#include "KraftKontrol/modules/navigation_modules/navigation_complementary.h"



void NavigationComplementaryFilter::thread() {

    //We will go through the history of sensor values to correct our model predictions.

    /*while (navigationData_.timestamp < micros()) {

        //Used to exit loop
        bool newData = false;

        //Used to determine oldest sensor value
        int64_t smallestTime = micros();

        //Gather valid sensor values
        Vector<> angularRateRaw; uint32_t angularRateTime = 0;
        if (gyro_->peekGyro(&angularRateRaw, &angularRateTime)) {
            if (angularRateTime < smallestTime) smallestTime = angularRateTime;
            newData = true;
        }

        Vector<> accelRaw; uint32_t accelTime = 0;
        if (accel_->peekAccel(&accelRaw, &accelTime)) {
            if (accelTime < smallestTime) smallestTime = accelTime;
            newData = true;
        }

        Vector<> magRaw; uint32_t magTime = 0;
        if ((mag_ != nullptr) ? mag_->peekMag(&magRaw, &magTime):false) {
            if (magTime < smallestTime) smallestTime = magTime;
            newData = true;
        }

        float baroRaw; uint32_t baroTime = 0;
        if ((baro_ != nullptr) ? baro_->peekPressure(&baroRaw, &baroTime):false) {
            if (baroTime < smallestTime) smallestTime = baroTime;
            newData = true;
        }

        Vector<> gnssVelRaw; uint32_t gnssVelTime = 0;
        if ((gnss_ != nullptr) ? gnss_->peekVelocity(&gnssVelRaw, &gnssVelTime):false) {
            if (gnssVelTime < smallestTime) smallestTime = gnssVelTime;
            newData = true;
        }

        Vector<> gnssPosRaw; uint32_t gnssPosTime = 0;
        if ((gnss_ != nullptr) ? gnss_->peekVelocity(&gnssPosRaw, &gnssPosTime):false) {
            if (gnssPosTime < smallestTime) smallestTime = gnssPosTime;
            newData = true;
        }


        //Predict system state until oldest sensor timestamp
        KinematicData statePrediction = predictState(navigationData_, smallestTime);


        //Correct state prediction with gyro values.
        if (angularRateTime != 0 && angularRateTime == smallestTime) {
            
            //Retreive gyro data
            gyro_->getGyro(&angularRateRaw, &angularRateTime);

            //Get delta time from last gyro data
            float dTime = float(angularRateTime - lastGyroTimestamp_)/1000000.0f;
            lastGyroTimestamp_ = angularRateTime;

            //Update filters
            if (angularRateRaw.magnitude() < 0.1f) angularRateRaw =- gyroLPF_.update(angularRateRaw);

            //update buffers
            gyroXBuffer_.placeFront(angularRateRaw.x, true);
            gyroYBuffer_.placeFront(angularRateRaw.y, true);
            gyroZBuffer_.placeFront(angularRateRaw.z, true);

            if (gyroXBuffer_.available() >= 2) {

                _gyroInitialized = true;

                //Calulate measured values
                ValueError<Vector<>> angularRate = ValueError<Vector<>>(Vector<>(gyroXBuffer_.getMedian(), gyroYBuffer_.getMedian(), gyroZBuffer_.getMedian()), Vector<>(gyroXBuffer_.getStandardError(), gyroYBuffer_.getStandardError(), gyroZBuffer_.getStandardError()));
                ValueError<Vector<>> angularAccel = (angularRate - lastAngularRateValue_)/dTime;

                //Calculate best prediction of current system
                ValueError<Vector<>> angularAccelBest = angularAccel.weightedAverage(ValueError<Vector<>>(statePrediction.attitude.rotateVector(statePrediction.angularAcceleration), statePrediction.attitude.rotateVector(statePrediction.angularAccelerationError)));
                ValueError<Vector<>> angularRateBest = angularRate.weightedAverage(ValueError<Vector<>>(statePrediction.attitude.rotateVector(statePrediction.angularRate), statePrediction.attitude.rotateVector(statePrediction.angularRateError)));

                //Predict next system state with measured values
                Vector<> angleChange = angularRate.value*dTime;//(navigationData_.angularAcceleration + angularAccelBest.value)*0.5*dTime*dTime + (navigationData_.angularRate + angularRateBest.value)*dTime;
                Quaternion<> attitude = navigationData_.attitude*Quaternion<>(angleChange, angleChange.magnitude());

                //Update state prediction
                statePrediction.attitude = attitude; // Sadly no error for this yet.

                statePrediction.angularRate = statePrediction.attitude.copy().conjugate().rotateVector(angularRateBest.value);
                statePrediction.angularRateError = statePrediction.attitude.copy().conjugate().rotateVector(angularRate.error);

                statePrediction.angularAcceleration = statePrediction.attitude.copy().conjugate().rotateVector(angularAccelBest.value);
                statePrediction.angularAccelerationError = statePrediction.attitude.copy().conjugate().rotateVector(angularAccelBest.error);

                //Update last values
                lastAngularRateValue_ = angularRate;

            }

        }


        //Correct state prediction with accel values
        if (accelTime != 0 && accelTime == smallestTime) {
            
            //Retreive gyro data
            accel_->getAccel(&accelRaw, &accelTime);

            //Get delta time from last gyro data
            float dTime = float(accelTime - lastAccelTimestamp_)/1000000.0f;
            lastAccelTimestamp_ = accelTime;

            //update buffers
            accelXBuffer_.placeFront(accelRaw.x, true);
            accelYBuffer_.placeFront(accelRaw.y, true);
            accelZBuffer_.placeFront(accelRaw.z, true);

            if (accelXBuffer_.available() >= 2) {

                float beta = 0.1f;

                //Z-Axis correction
                Vector<> zAxisIs = Vector<>(0,0,1);
                Vector<> zAxisSet = (statePrediction.attitude*accelRaw*statePrediction.attitude.copy().conjugate()).toVector();

                Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
                float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

                Quaternion<> zAxisCorrectionQuat = Quaternion<>(zAxisRotationAxis, zAxisRotationAngle*beta*dTime);


                //Apply state correction and normalise attitude quaternion 
                statePrediction.attitude = zAxisCorrectionQuat*statePrediction.attitude;
                statePrediction.attitude.normalize(true);


                //Update acceleration
                statePrediction.acceleration = (statePrediction.attitude*accelRaw*statePrediction.attitude.copy().conjugate()).toVector(); //Transform acceleration into world coordinate system and remove gravity
                Vector<> filtered = accelBiasLPF_.update(statePrediction.acceleration - Vector<>(0,0,9.81));
                statePrediction.linearAcceleration = statePrediction.acceleration - Vector<>(0,0,9.81) - filtered;

                //Update linear acceleration
                ValueError<> buf = ValueError<>(navigationData_.linearAcceleration.x, accelXBuffer_.getStandardError());
                navigationData_.acceleration.x = buf.value;
                navigationData_.accelerationError.x = buf.error;

                buf = ValueError<>(navigationData_.linearAcceleration.y, accelXBuffer_.getStandardError());
                navigationData_.acceleration.y = buf.value;
                navigationData_.accelerationError.y = buf.error;

                buf = ValueError<>(navigationData_.linearAcceleration.z, accelXBuffer_.getStandardError());
                navigationData_.acceleration.z = buf.value;
                navigationData_.accelerationError.z = buf.error;

                //Serial.println(String("Accel: x: ") + navigationData_.linearAcceleration.x + ", y: " + navigationData_.linearAcceleration.y + ", z: " + navigationData_.linearAcceleration.z);

            }

        }

        //Serial.println(String("Time: ") + micros() + ", smallest: " + uint32_t(smallestTime));

        //Update for next iteration
        navigationData_ = statePrediction;
        navigationData_.timestamp = smallestTime;

        if (!newData) break;


    }


    return;*/


    //Calculate time delta from last run
    float dTime = (float)(NOW() - lastLoopTimestamp_)/SECONDS;
    lastLoopTimestamp_ = NOW();

    //Predict current state and its error
    navigationData_.velocity += navigationData_.linearAcceleration*dTime;
    navigationData_.velocityError += navigationData_.accelerationError*dTime;

    navigationData_.position += navigationData_.velocity*dTime;
    navigationData_.absolutePosition.height = navigationData_.position.z + navigationData_.homePosition.height;
    navigationData_.positionError += navigationData_.velocityError*dTime;


    //Correct with sensor values
    while (gyroSub_.available() > 0) {
        
        //Get IMU data
        SensorTimestamp<Vector<>> sensorTime;
        gyroSub_.takeBack(&sensorTime);

        Vector<> rotationVector = sensorTime.sensorData;
        int64_t timestamp = sensorTime.sensorTimestamp;

        if (rotationVector.magnitude() < 0.1) {
            gyroLPF_.update(rotationVector);
        }
        rotationVector = rotationVector - gyroLPF_.getValue();

        gyroXBuffer_.placeFront(rotationVector.x, true);
        gyroYBuffer_.placeFront(rotationVector.y, true);
        gyroZBuffer_.placeFront(rotationVector.z, true);
        //rotationVector = Vector<>(gyroXBuffer_.getMedian(), gyroYBuffer_.getMedian(), gyroZBuffer_.getMedian());

        //Calulate time delta
        float dt = float(timestamp - lastGyroTimestamp_)/SECONDS;
        lastGyroTimestamp_ = timestamp;

        //Calulate derivitive of gyro for angular acceleration
        navigationData_.angularAcceleration = (rotationVector - lastGyroValue_)/dt;
        lastGyroValue_ = rotationVector;

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


    while (accelSub_.available() > 0) {

        //static Vector lastValue = 0;

        //Get IMU data
        SensorTimestamp<Vector<>> sensorTime;
        accelSub_.takeBack(&sensorTime);

        Vector<> accelVector = sensorTime.sensorData;
        int64_t timestamp = sensorTime.sensorTimestamp;

        //Serial.println(String("Accel: ") + accelVector.toString());

        //accelVector = accelLPF_.update(accelVector);

        accelXBuffer_.placeFront(accelVector.x, true);
        accelYBuffer_.placeFront(accelVector.y, true);
        accelZBuffer_.placeFront(accelVector.z, true);
        accelVector = Vector<>(accelXBuffer_.getAverage(), accelYBuffer_.getAverage(), accelZBuffer_.getAverage());

        //Serial.println(String("Accel: ") + accelVector.toString());

        //accelVector = lastValue = lastValue*0.9999 + accelVector*0.0001;

        //Serial.println("Accel: x: " + String(accelVector.x,4) + ", y: " + String(accelVector.y,4) + ", z: " + String(accelVector.z,4));

        //Check if accelerometer initialised
        if (_accelInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - lastAccelTimestamp_)/SECONDS;
            lastAccelTimestamp_ = timestamp;

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
            navigationData_.linearAcceleration = navigationData_.acceleration - Vector<>(0,0,9.81) - filtered;//accelHPF_.update(navigationData_.acceleration);

            //Update linear acceleration
            navigationData_.acceleration.x = navigationData_.linearAcceleration.x;
            navigationData_.accelerationError.x = accelXBuffer_.getStandardError();

            navigationData_.acceleration.y = navigationData_.linearAcceleration.y;
            navigationData_.accelerationError.y = accelYBuffer_.getStandardError();

            navigationData_.acceleration.z = navigationData_.linearAcceleration.z;
            navigationData_.accelerationError.z = accelZBuffer_.getStandardError();

            //Serial.println(String("Accel: x: ") + navigationData_.linearAcceleration.x + ", y: " + navigationData_.linearAcceleration.y + ", z: " + navigationData_.linearAcceleration.z);
            
            

        } else if (_gyroInitialized) {
            
            //Accel filter initialisation
            _accelInitialized = true;
            lastAccelTimestamp_ = timestamp;

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


    while (magSub_.available() > 0) {

        /*static Vector max = -1000;
        static Vector min = 1000;
        static Vector offset = 0;
        static Vector scale = 1;*/

        //Get IMU data
        SensorTimestamp<Vector<>> sensorTime;
        magSub_.takeBack(&sensorTime);

        Vector<>& magVector = sensorTime.sensorData;
        int64_t& timestamp = sensorTime.sensorTimestamp;

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

            //Mounting orientation compensation
            //magVector = Quaternion<>(Vector<>(0, -1, 0), 90*DEGREES).rotateVector(magVector);

            Vector<> magBuf = magVector;
            magVector.x = -magBuf.z;
            magVector.y = magBuf.x;
            magVector.z = magBuf.y;

            magVec_ = magVector;

            //Serial.println(String("Mag: ") + magVec_.toString());

            //Correct state prediction
            float dt = float(timestamp - lastMagTimestamp_)/SECONDS;
            lastMagTimestamp_ = timestamp;

            float gamma = 0.1f;

            //X-Axis correction
            Vector<> xAxisIs(1,0,0);
            Vector<> xAxisSet = (navigationData_.attitude*magVector*navigationData_.attitude.copy().conjugate()).toVector();
            xAxisSet.z = 0;

            Vector<> xAxisRotationAxis = Vector<>(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            Quaternion<> xAxisCorrectionQuat = Quaternion<>(xAxisRotationAxis, xAxisRotationAngle*gamma*dt);


            //Apply state correction and normalise attitude quaternion 
            navigationData_.attitude = xAxisCorrectionQuat*navigationData_.attitude;
            navigationData_.attitude.normalize(true);

        } else if (_accelInitialized) {

            //Magnetometer filter initialisation
            _magInitialized = true;
            lastMagTimestamp_ = timestamp;

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


    while (baroSub_.available() > 0) {

        SensorTimestamp<float> sensorTime;
        baroSub_.takeBack(&sensorTime);

        float& baroPressure_ = sensorTime.sensorData;
        int64_t& timestamp = sensorTime.sensorTimestamp;

        //Check if accelerometer initialised
        if (_baroInitialized) {
            
            //Correct state prediction
            float dt = float(timestamp - lastBaroTimestamp_)/SECONDS;
            lastBaroTimestamp_ = timestamp;

            //calculate height from new pressure value
            float heightAbsolute = getHeightFromPressure(baroPressure_, sealevelPressure_);
            //Serial.println(String("H: ") + heightAbsolute);
            //float heightRelative = heightAbsolute - navigationData_.absolutePosition.height;
            //calculate z velocity from new height value
            float zVelocity = (heightAbsolute - _lastHeightValue)/dt;
            _lastHeightValue = heightAbsolute;

            //Update buffers
            baroHeightBuffer_.placeFront(heightAbsolute, true);
            baroVelBuffer_.placeFront(zVelocity, true);

            float heightMedian = heightAbsolute;
            float heightError = baroHeightBuffer_.getStandardError()+0.20;

            float velMedian = zVelocity;
            float velError = baroVelBuffer_.getStandardError();

            //Serial.println(String("Height: ") + heightMedian + " +- " + String(heightError,5) + ",\tvel: " + velMedian + "+-" + velError);
            //Serial.println(String("") + (heightMedian - navigationData_.homePosition.height) + "  " + velMedian);

            //correct dead reckoning values with new ones.
            ValueError<> heightVel = ValueError<>(navigationData_.velocity.z, navigationData_.velocityError.z).weightedAverage(ValueError<>(velMedian, velError));
            navigationData_.velocity.z = heightVel.value;
            navigationData_.velocityError.z = heightVel.error;

            ValueError<> height = ValueError<>(navigationData_.absolutePosition.height, navigationData_.positionError.z).weightedAverage(ValueError<>(heightMedian, heightError));
            navigationData_.absolutePosition.height = height.value;
            navigationData_.positionError.z = height.error;

            //Update position z
            navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;

            


        } else {
            
            //Set flag to true
            _baroInitialized = true;
            
            //Barometer filter initialisation
            lastBaroTimestamp_ = timestamp;
            _lastHeightValue = getHeightFromPressure(baroPressure_, sealevelPressure_);

            baroHeightBuffer_.placeFront(_lastHeightValue, true);
            baroVelBuffer_.placeFront(0, true);

            //Set current calculated height as start value.
            navigationData_.absolutePosition.height = _lastHeightValue;

        }

    }

    

    while (gnssSub_.available() > 0) {

        GNSSData gnssData;
        gnssSub_.takeBack(&gnssData);
        
        if (gnssData.lockValid) {

            WorldPosition& positionAbsolute = gnssData.positionValueTimestamp.sensorData;
            int64_t& time = gnssData.positionValueTimestamp.sensorTimestamp;

            float& posError = gnssData.positionError;
            float& heightError = gnssData.altitudeError;

            navigationData_.absolutePosition.latitude = positionAbsolute.latitude;
            navigationData_.absolutePosition.longitude = positionAbsolute.longitude;
            //navigationData_.absolutePosition.height = positionAbsolute.height;

            Vector<> positionBuf = positionAbsolute.getPositionVectorFrom(navigationData_.homePosition);

            //Serial.println(navigationData_.homePosition.height);

            gnssPositionXBuffer_.placeFront(positionBuf.x, true);
            gnssPositionYBuffer_.placeFront(positionBuf.y, true);
            gnssPositionZBuffer_.placeFront(positionBuf.z, true);

            if (gnssPositionXBuffer_.available() >= 2) {

                ValueError<Vector<>> position;
                position.value = positionBuf;//Vector<>(gnssPositionXBuffer_.getMedian(), gnssPositionYBuffer_.getMedian(), gnssPositionZBuffer_.getMedian());
                position.error = Vector<>(posError, posError, heightError);

                //Create corrcted prediction
                position = ValueError<Vector<>>(Vector<>(navigationData_.position.x, navigationData_.position.y, navigationData_.absolutePosition.height), navigationData_.positionError).weightedAverage(position);

                if (!seaLevelPressureCorrected_ && baroPressure_ > 10) {
                    seaLevelPressureCorrected_ = true;
                    sealevelPressure_ = getSealevelPressureFromHeight(baroPressure_, positionAbsolute.height);
                } else if (seaLevelPressureCorrected_) {
                    //sealevelPressure_ = sealevelPressure_*0.99f + getSealevelPressureFromHeight(baroPressure_, positionAbsolute.height)*0.01f;
                }

                //Update position values.
                navigationData_.position.x = position.value.x;
                navigationData_.position.y = position.value.y;
                //navigationData_.position.z = position.value.z;// - navigationData_.homePosition.height;

                //navigationData_.absolutePosition.height = navigationData_.position.z + navigationData_.homePosition.height;

                navigationData_.positionError.x = position.error.x;
                navigationData_.positionError.y = position.error.y;
                //navigationData_.positionError.z = position.error.z;

            }


            Vector<>& velocityBuf = gnssData.velocityValueTimestamp.sensorData;
            time = gnssData.velocityValueTimestamp.sensorTimestamp;

            gnssVelocityXBuffer_.placeFront(velocityBuf.x, true);
            gnssVelocityYBuffer_.placeFront(velocityBuf.y, true);
            gnssVelocityZBuffer_.placeFront(velocityBuf.z, true);

            if (gnssVelocityXBuffer_.available() >= 2) {

                ValueError<Vector<>> velocity;
                velocity.value = Vector<>(gnssVelocityXBuffer_.getMedian(), gnssVelocityYBuffer_.getMedian(), gnssVelocityZBuffer_.getMedian());
                velocity.error = Vector<>(posError/10+0.02, posError/10+0.02, heightError/10+0.02);
                
                //Create corrcted prediction
                velocity = ValueError<Vector<>>(navigationData_.velocity, navigationData_.velocityError).weightedAverage(velocity);

                //Serial.println(String("Vel: ") + velocity.value.toString() + ", error: " + velocity.error.toString());

                //Update position values.
                //navigationData_.velocity.x = velocity.value.x;
                //navigationData_.velocity.y = velocity.value.y;
                //navigationData_.velocityError.x = velocity.error.x;
                //navigationData_.velocityError.y = velocity.error.y;

                navigationData_.velocity = velocity.value;
                navigationData_.velocityError = velocity.error;

            }

        }

    }

    navigationData_.timestamp = NOW();

    navigationDataTopic_.publish(navigationData_);

}



void NavigationComplementaryFilter::init() {

    navigationData_.angularAccelerationError = 10000;
    navigationData_.angularRateError = 10000;
    navigationData_.attitudeError = 10000;

    navigationData_.accelerationError = 10000;
    navigationData_.velocityError = 10000;
    navigationData_.positionError = 10000;

}
