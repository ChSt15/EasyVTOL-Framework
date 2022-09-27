#include "KraftKontrol/modules/navigation_modules/navigation_kalman_mix.h"



NavigationKalmanMix::NavigationKalmanMix(SystemModel_Abstract& systemModel) : Task_Abstract("Nav Kalmanfilter", 1000, eTaskPriority_t::eTaskPriority_High), systemModel_(systemModel) {}



void NavigationKalmanMix::thread() {

    //We will go through the history of sensor values.

    while (true) {

        //Used to exit loop
        bool newData = false;

        //Used to determine oldest sensor value
        int64_t smallestTime = NOW();

        //Gather valid sensor values
        gyroData.timestamp = 0;
        if (gyroSub_.isDataNew()) {
            firstGyroWait = true;
            gyroData = gyroSub_.getItem();
            if (gyroData.timestamp < smallestTime) smallestTime = gyroData.timestamp;
            newData = true;
        }

        accelData.timestamp = 0;
        if (accelSub_.isDataNew()) {
            firstAccelWait = true;
            accelData = accelSub_.getItem();
            if (accelData.timestamp < smallestTime) smallestTime = accelData.timestamp;
            newData = true;
        }

        magData.timestamp = 0;
        if (magSub_.isDataNew()) {
            magData = magSub_.getItem();
            if (magData.timestamp < smallestTime) smallestTime = magData.timestamp;
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

        magData.data.values(0) = 1;
        magData.data.values(1) = 0;
        magData.data.values(2) = 0;

        FML::Matrix33_F transform;
        transform(0,0) = 1;
        transform(0,1) = 0;
        transform(0,2) = 0;
        transform(1,0) = 0;
        transform(1,1) = -1;
        transform(1,2) = 0;
        transform(2,0) = 0;
        transform(2,1) = 0;
        transform(2,2) = -1;



        if (firstAccelWait && firstGyroWait) {
            if (attFilter.update(gyroData.data.values(0), -gyroData.data.values(1), -gyroData.data.values(2), accelData.data.values(0), -accelData.data.values(1), -accelData.data.values(2), magData.data.values(0), -magData.data.values(1), -magData.data.values(2))) attFilter.getQuaternion(&statePrediction.data.attitude.w, &statePrediction.data.attitude.x, &statePrediction.data.attitude.y, &statePrediction.data.attitude.z);
        }




        //Correct state prediction with gyro values.
        if (gyroData.timestamp != 0 && gyroData.timestamp == smallestTime) {

            //Get delta time from last gyro data
            float dTime = float(gyroData.timestamp - lastGyroData_.timestamp)/SECONDS;

            //Serial.println(String("Gyro: ") + angularRateRaw.data.x + ", y: " + angularRateRaw.data.y + ", z: " + angularRateRaw.data.z);

            //Change to world coordinate system
            Vector<> angRateRawWorld = statePrediction.data.attitude.rotateVector(Vector<>(gyroData.data.values(0), gyroData.data.values(1), gyroData.data.values(2)));

            
            //Serial.print(String("cov: ") + gyroCov.x + ", " + gyroCov.y + ", " + gyroCov.z);


            //Calculate angular acceleration value and error
            auto gyroAccel = (gyroData.data.values - lastGyroData_.data.values)/dTime;
            auto gyroAccelCov = (gyroData.data.covariance + lastGyroData_.data.covariance)/dTime;


            //Correct state prediction
            //Vector<> accelGain = statePrediction.data.angularAccelerationError/(statePrediction.data.angularAccelerationError + gyroAccelCov);
            statePrediction.data.angularAcceleration = Vector<>(gyroData.data.values(0), gyroData.data.values(1), gyroData.data.values(2));//statePrediction.data.angularAcceleration + accelGain*(gyroAccel-statePrediction.data.angularAcceleration);

            //Vector<> rateGain = statePrediction.data.angularRateError/(statePrediction.data.angularRateError + gyroCov);
            statePrediction.data.angularRate = angRateRawWorld;//statePrediction.data.angularRate + rateGain*(angularRateRaw.data-statePrediction.data.angularRate);
            
            //Calculate new state errors
            statePrediction.data.angularAccelerationError = Vector<>(gyroAccelCov(0,0), gyroAccelCov(1,1), gyroAccelCov(2,2));//(Vector<>(1)-accelGain)*statePrediction.data.angularAccelerationError;
            statePrediction.data.angularRateError = Vector<>(gyroData.data.covariance(0,0), gyroData.data.covariance(1,1), gyroData.data.covariance(2,2));//(Vector<>(1)-rateGain)*statePrediction.data.angularRateError;

            //Serial.println(String(", Gyro: ") + navigationData_.data.angularRate.x + ", " + navigationData_.data.angularRate.y + ", " + navigationData_.data.angularRate.z);

            //if (!(rateGain.z == rateGain.z)) while(1);

            //Update old values
            lastGyroData_ = gyroData;

        }


        //Correct state prediction with accel values
        if (accelData.timestamp != 0 && accelData.timestamp == smallestTime) {

            //Get delta time from last accel data
            float dTime = float(accelData.timestamp - lastAccelTimestamp_)/SECONDS;

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

            //Serial.println(String("Accel: ") + accelCov.x + ", " + accelCov.y + ", " + accelCov.z);

            


            //Rotate to world coordinate system.
            Vector<> worldAccel = statePrediction.data.attitude.rotateVector(Vector<>(accelData.data.values(0), accelData.data.values(1), accelData.data.values(2)));
            //accelCov = statePrediction.data.attitude.rotateVector(accelCov);

            //Run kalman filter correction stuff 
            //Vector<> accelGain = statePrediction.data.accelerationError/(statePrediction.data.accelerationError + accelCov);
            statePrediction.data.acceleration = worldAccel;//statePrediction.data.acceleration + accelGain*(accelRaw.data-statePrediction.data.acceleration);

            statePrediction.data.linearAcceleration = statePrediction.data.acceleration + GRAVITY_VECTOR;


            /*float freq = 1.0f/(abs(accelRaw.data.magnitude() - GRAVITY_MAGNITUDE)+1);
            freq *= freq;
            accelBias.setParameters(0.03*freq);
            accelBias.update(statePrediction.data.attitude.rotateVector(statePrediction.data.linearAcceleration));

            Serial.println(String("Bias: x: ") + accelBias.getValue().x +  ", y: " + accelBias.getValue().y +  ", z: " + accelBias.getValue().z);*/
            
            //Calculate new state errors
            statePrediction.data.accelerationError = Vector<>(accelData.data.covariance(0,0), accelData.data.covariance(1,1), accelData.data.covariance(2,2));//(Vector<>(1)-accelGain)*statePrediction.data.accelerationError;

            //Serial.println(String("Cov: x: ") + abs(accelCov.x) + ", y: " + abs(accelCov.y) + ", z: " + abs(accelCov.z));

            //Update absolute position.
            statePrediction.data.absolutePosition.height = statePrediction.data.position.z + statePrediction.data.homePosition.height;

            /*Serial.print(String("Pred: ") + statePrediction.data.acceleration.x + ", " + statePrediction.data.acceleration.y + ", " + statePrediction.data.acceleration.z);
            Serial.print(String(", Meas: ") + accelRaw.data.x + ", " + accelRaw.data.y + ", " + accelRaw.data.z);
            Serial.println();*/

            //if (!(accelGain.z == accelGain.z)) while(1);


            //Update old values
            lastAccelTimestamp_ = accelData.timestamp;
            

        }


        //Correct state prediction with mag values
        if (magData.timestamp != 0 && magData.timestamp == smallestTime) {

            //Get delta time from last gyro data
            float dTime = float(magData.timestamp - lastMagTimestamp_)/SECONDS;

            //Serial.println(String("Mag: ") + magRaw.data.x + ", " + magRaw.data.y + ", " + magRaw.data.z);

            //Find out mag covariance and value in world coordinates
            Vector<> magWorld = statePrediction.data.attitude.rotateVector(Vector<>(magData.data.values(0), magData.data.values(1), magData.data.values(2)));
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
            Vector<> xAxisSet = magWorld;
            xAxisSet.z = 0;
            xAxisSet.normalize();

            Vector<> xAxisRotationAxis = Vector<>(0,0,1);
            float xAxisRotationAngle = -atan2(xAxisSet.y, xAxisSet.x);

            FML::Quaternion<> xAxisCorrectionQuat = FML::Quaternion<>(xAxisRotationAxis, xAxisRotationAngle*gamma*dTime);


            //Apply state correction and normalise attitude quaternion 
            statePrediction.data.attitude = xAxisCorrectionQuat*statePrediction.data.attitude;
            statePrediction.data.attitude.normalize(true);

            lastMagTimestamp_ = magData.timestamp;
            

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



void NavigationKalmanMix::setGyroscopeInput(const Gyroscope_Abstract& gyro) {
    gyroSub_.subscribe(gyro.getGyroTopic());
}



void NavigationKalmanMix::removeGyroscopeInput() {
    gyroSub_.unsubcribe();
}



void NavigationKalmanMix::setAccelerometerInput(const Accelerometer_Abstract& accel) {
    accelSub_.subscribe(accel.getAccelTopic());
}



void NavigationKalmanMix::removeAccelerometerInput() {
    accelSub_.unsubcribe();
}



void NavigationKalmanMix::setMagnetometerInput(const Magnetometer_Abstract& mag) {
    magSub_.subscribe(mag.getMagTopic());
}



void NavigationKalmanMix::removeMagnetometerInput() {
    magSub_.unsubcribe();
}



void NavigationKalmanMix::setBarometerInput(const Barometer_Abstract& baro) {
    baroSub_.subscribe(baro.getBaroTopic());
}



void NavigationKalmanMix::removeBarometerInput() {
    baroSub_.unsubcribe();
}



void NavigationKalmanMix::setGNSSInput(const GNSS_Abstract& gnss) {
    gnssSub_.subscribe(gnss.getGNSSTopic());
}



void NavigationKalmanMix::removeGNSSInput() {
    gnssSub_.unsubcribe();
}



void NavigationKalmanMix::init() {

    navigationData_.data.angularAccelerationError = 10000000;
    navigationData_.data.angularRateError = 10000000;
    navigationData_.data.attitudeError = 10000000;

    navigationData_.data.accelerationError = 10000000;
    navigationData_.data.velocityError = 10000000;
    navigationData_.data.positionError = 10000000;

    attFilter.setInitializationDuration(20000000);

}
