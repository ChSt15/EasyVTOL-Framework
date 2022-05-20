#include "KraftKontrol/modules/navigation_modules/navigation_kalman.h"



NavigationKalman::NavigationKalman(SystemModel_Abstract& systemModel) : Task_Threading("Nav Kalmanfilter", eTaskPriority_t::eTaskPriority_High, SECONDS/1000), systemModel_(systemModel) {}



void NavigationKalman::thread() {

    //We will go through the history of sensor values.

    while (true) {

        //Used to exit loop
        bool newData = false;

        //Used to determine oldest sensor value
        int64_t smallestTime = NOW();

        //Gather valid sensor values
        DataTimestamped<Vector<>> angularRateRaw;
        if (gyroSub_.isDataNew()) {
            angularRateRaw.data.x = gyroSub_.getItem().data.values[0][0];
            angularRateRaw.data.y = gyroSub_.getItem().data.values[1][0];
            angularRateRaw.data.z = gyroSub_.getItem().data.values[2][0];
            angularRateRaw.timestamp = gyroSub_.getItem().timestamp;
            if (angularRateRaw.timestamp < smallestTime) smallestTime = angularRateRaw.timestamp;
            newData = true;
        }

        DataTimestamped<Vector<>> accelRaw;
        if (accelSub_.isDataNew()) {
            accelRaw.data.x = accelSub_.getItem().data.values[0][0];
            accelRaw.data.y = accelSub_.getItem().data.values[1][0];
            accelRaw.data.z = accelSub_.getItem().data.values[2][0];
            accelRaw.timestamp = accelSub_.getItem().timestamp;
            if (accelRaw.timestamp < smallestTime) smallestTime = accelRaw.timestamp;
            newData = true;
        }

        DataTimestamped<Vector<>> magRaw;
        if (magSub_.isDataNew()) {
            magRaw.data.x = magSub_.getItem().data.values[0][0];
            magRaw.data.y = magSub_.getItem().data.values[1][0];
            magRaw.data.z = magSub_.getItem().data.values[2][0];
            magRaw.timestamp = magSub_.getItem().timestamp;
            if (magRaw.timestamp < smallestTime) smallestTime = magRaw.timestamp;
            newData = true;
        }

        DataTimestamped<float> baroRaw;
        if (baroSub_.isDataNew()) {
            baroRaw = baroSub_.getItem();
            if (baroRaw.timestamp < smallestTime) smallestTime = baroRaw.timestamp;
            newData = true;
        }

        DataTimestamped<GNSSData> gnssRaw;
        if (gnssSub_.isDataNew()) {
            gnssRaw = gnssSub_.getItem();
            if (gnssRaw.timestamp < smallestTime) smallestTime = gnssRaw.timestamp;
            newData = true;
        }


        //If no new sensor data available then just predict till current time and leave the loop.
        if (!newData) {

            systemModel_.predictState(navigationData_, NOW());

            break; //Loop ends here!

        }


        //Predict system state until oldest sensor timestamp
        DataTimestamped<NavigationData> statePrediction(navigationData_);
        systemModel_.predictState(statePrediction, smallestTime);


        //Correct state prediction with gyro values.
        if (angularRateRaw.timestamp != 0 && angularRateRaw.timestamp == smallestTime) {

            //Get delta time from last gyro data
            float dTime = float(angularRateRaw.timestamp - lastGyroTimestamp_)/SECONDS;

            //Serial.println(String("Gyro: ") + angularRateRaw.data.x + ", y: " + angularRateRaw.data.y + ", z: " + angularRateRaw.data.z);

            //Update filters
            float beta = 1.0/(abs(angularRateRaw.data.magnitude() - GRAVITY_MAGNITUDE)+1); 
            beta = 0.1*beta*beta;
            gyroBiasLPF_.setParameters(beta);
            //angularRateRaw.data -= gyroBiasLPF_.update(angularRateRaw.data);

            //Change to world coordinate system
            angularRateRaw.data = statePrediction.data.attitude.rotateVector(angularRateRaw.data);

            //Calulate gyro error
            Vector<> gyroCov = (angularRateRaw.data-statePrediction.data.angularRate).square();

            angularRateCovBuf_.placeFront(gyroCov, true);
            gyroCov = 0;
            for (uint32_t i = 0; i < angularRateCovBuf_.available(); i++) {
                gyroCov += angularRateCovBuf_[i];
            }
            gyroCov = sqrt(gyroCov/angularRateCovBuf_.available());

            //Serial.print(String("cov: ") + gyroCov.x + ", " + gyroCov.y + ", " + gyroCov.z);


            //Calculate angular acceleration value and error
            Vector<> gyroAccel = (angularRateRaw.data-lastAngularRateValue_)/dTime;
            Vector<> gyroAccelCov = (gyroCov+lastAngularRateCov_)/dTime;


            //Correct state prediction
            //Vector<> accelGain = statePrediction.data.angularAccelerationError/(statePrediction.data.angularAccelerationError + gyroAccelCov);
            statePrediction.data.angularAcceleration = gyroAccel;//statePrediction.data.angularAcceleration + accelGain*(gyroAccel-statePrediction.data.angularAcceleration);

            //Vector<> rateGain = statePrediction.data.angularRateError/(statePrediction.data.angularRateError + gyroCov);
            statePrediction.data.angularRate = angularRateRaw.data;//statePrediction.data.angularRate + rateGain*(angularRateRaw.data-statePrediction.data.angularRate);
            
            //Calculate new state errors
            statePrediction.data.angularAccelerationError = gyroAccelCov;//(Vector<>(1)-accelGain)*statePrediction.data.angularAccelerationError;
            statePrediction.data.angularRateError = gyroCov;//(Vector<>(1)-rateGain)*statePrediction.data.angularRateError;

            //Serial.println(String(", Gyro: ") + navigationData_.data.angularRate.x + ", " + navigationData_.data.angularRate.y + ", " + navigationData_.data.angularRate.z);

            //if (!(rateGain.z == rateGain.z)) while(1);

            //Update old values
            lastAngularRateValue_ = angularRateRaw.data;
            lastAngularRateCov_ = gyroCov;
            lastGyroTimestamp_ = angularRateRaw.timestamp;

        }


        //Correct state prediction with accel values
        if (accelRaw.timestamp != 0 && accelRaw.timestamp == smallestTime) {

            //Get delta time from last accel data
            float dTime = float(accelRaw.timestamp - lastAccelTimestamp_)/SECONDS;

            //static LowPassFilter<Vector<>> LPF = 0.1;
            //Vector<> value = LPF.update(accelRaw.data);

            //Serial.println(String("Val: ") + String(value.x, 4) + ", " + String(value.y, 4) + ", " + String(value.z, 4));

            //Serial.println(String("Val: ") + String(accelRaw.data.x, 4) + ", " + String(accelRaw.data.y, 4) + ", " + String(accelRaw.data.z, 4));

            //Remove accel bias.
            //accelRaw.data -= accelBias.getValue();

            //Find out accel covariance and value in world coordinates
            //accelRaw.data = statePrediction.data.attitude.rotateVector(accelRaw.data);
            //Vector<> accelMean = (lastAccelValue_ + accelRaw.data)*0.5;
            /*Vector<> accelCov = (accelRaw.data-statePrediction.data.attitude.copy().conjugate().rotateVector(statePrediction.data.acceleration)).square();
            lastAccelValue_ = accelRaw.data;

            accelCovBuf_.placeFront(accelCov, true);
            accelCov = 0;
            for (uint32_t i = 0; i < accelCovBuf_.available(); i++) {
                accelCov += accelCovBuf_[i];
            }
            accelCov = sqrt(accelCov/accelCovBuf_.available());*/

            Vector<> accelCov = Vector<>(0.03);

            //Serial.println(String("Accel: ") + accelCov.x + ", " + accelCov.y + ", " + accelCov.z);

            //Fusion below for attitude is still a complementary filter.
            float beta = 1.0/(abs(accelRaw.data.magnitude() - GRAVITY_MAGNITUDE)+1); 
            beta = 0.1*beta*beta;

            //Z-Axis correction
            Vector<> zAxisIs = Vector<>(0,0,1);
            Vector<> zAxisSet = statePrediction.data.attitude.rotateVector(accelRaw.data);

            Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            FML::Quaternion<> zAxisCorrectionQuat = FML::Quaternion<>(zAxisRotationAxis, zAxisRotationAngle*beta*dTime);


            //Apply state correction and normalise attitude quaternion 
            statePrediction.data.attitude = zAxisCorrectionQuat*statePrediction.data.attitude;
            statePrediction.data.attitude.normalize(true);


            //Rotate to world coordinate system.
            accelRaw.data = statePrediction.data.attitude.rotateVector(accelRaw.data);
            //accelCov = statePrediction.data.attitude.rotateVector(accelCov);

            //Run kalman filter correction stuff 
            //Vector<> accelGain = statePrediction.data.accelerationError/(statePrediction.data.accelerationError + accelCov);
            statePrediction.data.acceleration = accelRaw.data;//statePrediction.data.acceleration + accelGain*(accelRaw.data-statePrediction.data.acceleration);

            statePrediction.data.linearAcceleration = statePrediction.data.acceleration + GRAVITY_VECTOR;


            /*float freq = 1.0f/(abs(accelRaw.data.magnitude() - GRAVITY_MAGNITUDE)+1);
            freq *= freq;
            accelBias.setParameters(0.03*freq);
            accelBias.update(statePrediction.data.attitude.rotateVector(statePrediction.data.linearAcceleration));

            Serial.println(String("Bias: x: ") + accelBias.getValue().x +  ", y: " + accelBias.getValue().y +  ", z: " + accelBias.getValue().z);*/
            
            //Calculate new state errors
            statePrediction.data.accelerationError = Vector<>(0.03);//(Vector<>(1)-accelGain)*statePrediction.data.accelerationError;

            //Serial.println(String("Cov: x: ") + abs(accelCov.x) + ", y: " + abs(accelCov.y) + ", z: " + abs(accelCov.z));

            //Update absolute position.
            statePrediction.data.absolutePosition.height = statePrediction.data.position.z + statePrediction.data.homePosition.height;

            /*Serial.print(String("Pred: ") + statePrediction.data.acceleration.x + ", " + statePrediction.data.acceleration.y + ", " + statePrediction.data.acceleration.z);
            Serial.print(String(", Meas: ") + accelRaw.data.x + ", " + accelRaw.data.y + ", " + accelRaw.data.z);
            Serial.println();*/

            //if (!(accelGain.z == accelGain.z)) while(1);


            //Update old values
            lastAccelCov_ = accelCov;
            lastAccelTimestamp_ = accelRaw.timestamp;
            

        }


        //Correct state prediction with mag values
        if (magRaw.timestamp != 0 && magRaw.timestamp == smallestTime) {

            //Get delta time from last gyro data
            float dTime = float(magRaw.timestamp - lastMagTimestamp_)/SECONDS;

            //Serial.println(String("Mag: ") + magRaw.data.x + ", " + magRaw.data.y + ", " + magRaw.data.z);

            //Find out mag covariance and value in world coordinates
            magRaw.data = statePrediction.data.attitude.rotateVector(magRaw.data);
            //Vector<> magCov = (magRaw.data-statePrediction.data.attitude.rotateVector(Vector<>(1,0,0))).square();

            /*accelCovBuf_.placeFront(accelCov, true);
            accelCov = 0;
            for (uint32_t i = 0; i < accelCovBuf_.available(); i++) {
                accelCov += accelCovBuf_[i];
            }
            accelCov = sqrt(accelCov/accelCovBuf_.available());*/

            float gamma = 0.1;

            //X-Axis correction
            Vector<> xAxisIs(1,0,0);
            Vector<> xAxisSet = magRaw.data;
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector<> xAxisRotationAxis = Vector<>(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            FML::Quaternion<> xAxisCorrectionQuat = FML::Quaternion<>(xAxisRotationAxis, xAxisRotationAngle*gamma*dTime);


            //Apply state correction and normalise attitude quaternion 
            statePrediction.data.attitude = xAxisCorrectionQuat*statePrediction.data.attitude;
            statePrediction.data.attitude.normalize(true);

            //Fusion below for attitude is still a complementary filter.
            float beta = 0.1f; 

            //Z-Axis correction
            Vector<> zAxisIs = Vector<>(0,0,1);
            Vector<> zAxisSet = statePrediction.data.attitude.rotateVector(accelRaw.data);

            Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            FML::Quaternion<> zAxisCorrectionQuat = FML::Quaternion<>(zAxisRotationAxis, zAxisRotationAngle*beta*dTime);


            //Apply state correction and normalise attitude quaternion 
            statePrediction.data.attitude = zAxisCorrectionQuat*statePrediction.data.attitude;
            statePrediction.data.attitude.normalize(true);



            //Update old values
            lastMagValue_ = magRaw.data;
            //lastMagCov_ = magCov;
            lastMagTimestamp_ = magRaw.timestamp;
            

        }


        //Correct state prediction with baro values
        if (baroRaw.timestamp != 0 && baroRaw.timestamp == smallestTime) {

            //Get delta time from last accel data
            float dTime = float(baroRaw.timestamp - lastBaroTimestamp_)/SECONDS;

            //calculate height from new pressure value
            float heightAbsolute = Barometer_Abstract::getHeightFromPressure(baroRaw.data, sealevelPressure_);

            /*float baroCov = heightAbsolute;
            //baroCov = baroCov*baroCov;

            baroCovBuf_.placeFront(baroCov, true);
            baroCov = 0;
            for (uint32_t i = 0; i < baroCovBuf_.available(); i++) {
                baroCov += baroCovBuf_[i];
            }
            baroCov = baroCovBuf_.getStandardDeviation();//sqrt(baroCov/baroCovBuf_.available());*/

            float baroCov = 1.0;

            //Serial.println(String("Baro: ") + baroCov);

            //calculate z velocity from new height value
            float zVelocity = (heightAbsolute - lastBaroValue_)/dTime;
            float zVelCov = (baroCov + lastBaroCov_)/dTime;

            //Serial.println(String(navigationData_.data.position.x, 4) + "," + String(navigationData_.data.positionError.x, 4) + "," + String(navigationData_.data.position.y, 4) + "," + String(navigationData_.data.positionError.y, 4) + "," + String(heightAbsolute, 4) + "," + String(heightError, 4));

            //Serial.println(String("Height: ") + heightAbsolute + " +- " + String(heightError,5) + ", velMed: " + velMedian + ",\tvel: " + zVelocity + "+-" + velError);
            //Serial.println(String("") + (heightMedian - navigationData_.data.homePosition.height) + "  " + velMedian);

            //Serial.println(String("Height: ") + heightMedian + " +- " + heightError + ", vel: ")

            //static LowPassFilter<float> velLPF = 0.05;
            //static LowPassFilter<float> hLPF = 0.05;

            //Serial.println(String("H: ") + hLPF.update(heightAbsolute-statePrediction.data.homePosition.height) + " V: " + velLPF.update(zVelocity));


            //Run kalman filter correction stuff 
            float heightGain = statePrediction.data.positionError.z*100/(statePrediction.data.positionError.z*100 + baroCov*100);
            statePrediction.data.absolutePosition.height = statePrediction.data.absolutePosition.height + heightGain*(heightAbsolute - statePrediction.data.absolutePosition.height);

            //float posError = statePrediction.data.positionError.z;
            
            //Calculate new state errors
            statePrediction.data.positionError.z = (1.0f-heightGain)*statePrediction.data.positionError.z;

            //Serial.println(String("H: ") + (heightAbsolute-statePrediction.data.homePosition.height) + " V: " + zVelocity + ", G: " + heightGain + ", PE: " + posError + ", NPE: " + statePrediction.data.positionError.z + ", PES: " + statePrediction.data.absolutePosition.height + ", time: " + int32_t(NOW()/SECONDS));

            float zVelGain = statePrediction.data.velocityError.z*100/(statePrediction.data.velocityError.z*100 + zVelCov*100);
            statePrediction.data.velocity.z = statePrediction.data.velocity.z + zVelGain*(zVelocity - statePrediction.data.velocity.z);
            
            //Calculate new state errors
            statePrediction.data.velocityError.z = (1.0f-zVelGain)*statePrediction.data.velocityError.z;

            /*Serial.print(String("Pred: ") + statePrediction.data.acceleration.x + ", " + statePrediction.data.acceleration.y + ", " + statePrediction.data.acceleration.z);
            Serial.print(String(", Meas: ") + accelRaw.data.x + ", " + accelRaw.data.y + ", " + accelRaw.data.z);
            Serial.println();*/

            //if (!(accelGain.z == accelGain.z)) while(1);

            //Update position z
            statePrediction.data.position.z = statePrediction.data.absolutePosition.height - statePrediction.data.homePosition.height;


            //Update old values
            lastBaroValue_ = heightAbsolute;
            lastBaroCov_ = baroCov;
            lastBaroTimestamp_ = baroRaw.timestamp;
            

        }


        //Correct state prediction with gnss values
        if (gnssRaw.timestamp != 0 && gnssRaw.timestamp == smallestTime) {

            //Get delta time from last accel data
            float dTime = float(gnssRaw.timestamp - lastGNSSTimestamp_)/SECONDS;

            //Only update if the lock is valid
            if (gnssRaw.data.lockValid) {


                statePrediction.data.absolutePosition.latitude = gnssRaw.data.position.latitude;
                statePrediction.data.absolutePosition.longitude = gnssRaw.data.position.longitude;
                //navigationData_.data.absolutePosition.height = positionAbsolute.height;

                Vector<> positionBuf = gnssRaw.data.position.getPositionVectorFrom(statePrediction.data.homePosition);


                //Run kalman filter correction stuff for pos
                Vector<> posGain = 1;// statePrediction.data.positionError/(statePrediction.data.positionError + gnssRaw.data.positionError);
                statePrediction.data.position.x = statePrediction.data.position.x + posGain.x*(positionBuf.x - statePrediction.data.position.x);
                statePrediction.data.position.y = statePrediction.data.position.y + posGain.y*(positionBuf.y - statePrediction.data.position.y);

                
                //Calculate new state errors
                statePrediction.data.positionError.x = (1.0f-posGain.x)*statePrediction.data.positionError.x;
                statePrediction.data.positionError.y = (1.0f-posGain.y)*statePrediction.data.positionError.y;


                //Run kalman filter correction stuff for vel
                Vector<> velGain = 1;// statePrediction.data.velocityError/(statePrediction.data.velocityError + gnssRaw.data.velocityError);
                statePrediction.data.velocity.x = statePrediction.data.velocity.x + posGain.x*(gnssRaw.data.velocity.x - statePrediction.data.velocity.x);
                statePrediction.data.velocity.y = statePrediction.data.velocity.y + posGain.y*(gnssRaw.data.velocity.y - statePrediction.data.velocity.y);

                
                //Calculate new state errors
                statePrediction.data.velocityError.x = (1.0f-posGain.x)*statePrediction.data.velocityError.x;
                statePrediction.data.velocityError.y = (1.0f-posGain.y)*statePrediction.data.velocityError.y;

                /*Serial.print(String("Pred: ") + statePrediction.data.acceleration.x + ", " + statePrediction.data.acceleration.y + ", " + statePrediction.data.acceleration.z);
                Serial.print(String(", Meas: ") + accelRaw.data.x + ", " + accelRaw.data.y + ", " + accelRaw.data.z);
                Serial.println();*/

                //if (!(accelGain.z == accelGain.z)) while(1);

            }


            //Update old values
            lastGNSSTimestamp_ = gnssRaw.timestamp;
            

        }

        //Serial.println(String("Time: ") + micros() + ", smallest: " + uint32_t(smallestTime));

        //Serial.println(String("a: ") + statePrediction.data.accelerationError.z + ", v: " + statePrediction.data.velocityError.z + ", p: "  + statePrediction.data.positionError.z);

        //Update for next iteration
        statePrediction.timestamp = smallestTime;
        navigationData_ = statePrediction;

    }

    navigationDataTopic_.publish(navigationData_);

}



void NavigationKalman::setGyroscopeInput(const Gyroscope_Abstract& gyro) {
    gyroSub_.subscribe(gyro.getGyroTopic());
}



void NavigationKalman::removeGyroscopeInput() {
    gyroSub_.unsubcribe();
}



void NavigationKalman::setAccelerometerInput(const Accelerometer_Abstract& accel) {
    accelSub_.subscribe(accel.getAccelTopic());
}



void NavigationKalman::removeAccelerometerInput() {
    accelSub_.unsubcribe();
}



void NavigationKalman::setMagnetometerInput(const Magnetometer_Abstract& mag) {
    magSub_.subscribe(mag.getMagTopic());
}



void NavigationKalman::removeMagnetometerInput() {
    magSub_.unsubcribe();
}



void NavigationKalman::setBarometerInput(const Barometer_Abstract& baro) {
    baroSub_.subscribe(baro.getBaroTopic());
}



void NavigationKalman::removeBarometerInput() {
    baroSub_.unsubcribe();
}



void NavigationKalman::setGNSSInput(const GNSS_Abstract& gnss) {
    gnssSub_.subscribe(gnss.getGNSSTopic());
}



void NavigationKalman::removeGNSSInput() {
    gnssSub_.unsubcribe();
}



void NavigationKalman::init() {

    navigationData_.data.angularAccelerationError = 10000000;
    navigationData_.data.angularRateError = 10000000;
    navigationData_.data.attitudeError = 10000000;

    navigationData_.data.accelerationError = 10000000;
    navigationData_.data.velocityError = 10000000;
    navigationData_.data.positionError = 10000000;

}
