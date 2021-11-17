#include "KraftKontrol/modules/navigation_modules/navigation_kalman.h"



NavigationKalman::NavigationKalman() : Task_Abstract("Nav Kalmanfilter", 1000, eTaskPriority_t::eTaskPriority_High) {}



void NavigationKalman::predictState(DataTimestamped<NavigationData>& systemState, int64_t time) {

    //Time saving time calculations
    float dTime = float(time-systemState.timestamp)/SECONDS;

    float dTimePow2 = dTime*dTime;
    float dTimeHalfPow2 = dTimePow2*0.5f;
    float dTimeQuartPow4 = dTimePow2*dTimePow2*0.25f;


    //Collect info on actuators
    DynamicData dynamicState = Actuator_Abstract::getSumDynamics();

    //Calculate resulting accelerations
    //systemState.data.angularAcceleration = systemState.data.attitude.rotateVector(dynamicState.torqe)/vehicleMomentInertia;
    systemState.data.angularAccelerationError = systemState.data.angularAccelerationError + 10*dTime;

    //Predict angular stuff
    systemState.data.angularRate = systemState.data.angularRate + systemState.data.angularAcceleration*dTime; //Integrate angular accel to get new angular rate
    systemState.data.angularRateError = systemState.data.angularAccelerationError*dTime + systemState.data.angularRateError; //Calculate new error

    Vector<> angleChange = systemState.data.angularAcceleration*dTimeHalfPow2 + systemState.data.angularRate*dTime;
    systemState.data.attitude = Quaternion<>(angleChange, angleChange.magnitude())*systemState.data.attitude;

    //Calculate resulting accelerations
    //systemState.data.acceleration = GRAVITY_VECTOR + systemState.data.attitude.rotateVector(dynamicState.force)/vehicleMass;
    //systemState.data.accelerationError = systemState.data.accelerationError + 10*dTime;

    //Predict positional stuff
    systemState.data.velocity = systemState.data.velocity + systemState.data.linearAcceleration*dTime; //Integrate linear accel for velocity
    systemState.data.velocityError = systemState.data.accelerationError*dTime + systemState.data.velocityError; //Calculate new error

    systemState.data.position = systemState.data.position + systemState.data.velocity*dTime + systemState.data.linearAcceleration*dTimeHalfPow2; //Integrate linear accel for velocity
    systemState.data.positionError = systemState.data.accelerationError*dTimeHalfPow2 + systemState.data.velocityError*dTime + systemState.data.positionError; //Calculate new error


    //Update time
    systemState.timestamp = time;

}



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
            angularRateRaw = gyroSub_.getItem();
            if (angularRateRaw.timestamp < smallestTime) smallestTime = angularRateRaw.timestamp;
            newData = true;
        }

        DataTimestamped<Vector<>> accelRaw;
        if (accelSub_.isDataNew()) {
            accelRaw = accelSub_.getItem();
            if (accelRaw.timestamp < smallestTime) smallestTime = accelRaw.timestamp;
            newData = true;
        }

        DataTimestamped<Vector<>> magRaw;
        if (magSub_.isDataNew()) {
            magRaw = magSub_.getItem();
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

            predictState(navigationData_, NOW());

            break; //Loop ends here!

        }


        //Predict system state until oldest sensor timestamp
        DataTimestamped<NavigationData> statePrediction(navigationData_);
        predictState(statePrediction, smallestTime);


        //Correct state prediction with gyro values.
        if (angularRateRaw.timestamp != 0 && angularRateRaw.timestamp == smallestTime) {
            
            //Remove oldest gyro data
            //gyroSub_.removeBack();

            //Get delta time from last gyro data
            float dTime = float(angularRateRaw.timestamp - lastGyroTimestamp_)/SECONDS;

            //Update filters
            //if (angularRateRaw.magnitude() < 0.1f) angularRateRaw =- gyroLPF_.update(angularRateRaw);

            //update buffers
            /*gyroXBuffer_.placeFront(angularRateRaw.x, true);
            gyroYBuffer_.placeFront(angularRateRaw.y, true);
            gyroZBuffer_.placeFront(angularRateRaw.z, true);*/


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

            Serial.print(String("cov: ") + gyroCov.x + ", " + gyroCov.y + ", " + gyroCov.z);


            //Calculate angular acceleration value and error
            Vector<> gyroAccel = (angularRateRaw.data-lastAngularRateValue_)/max(dTime, 0.0001);
            Vector<> gyroAccelCov = (gyroCov+lastAngularRateCov_)/max(dTime, 0.0001);


            //Correct state prediction
            Vector<> accelGain = statePrediction.data.angularAccelerationError/(statePrediction.data.angularAccelerationError + gyroAccelCov);
            navigationData_.data.angularAcceleration = gyroAccel;//statePrediction.data.angularAcceleration + accelGain*(gyroAccel-statePrediction.data.angularAcceleration);

            Vector<> rateGain = statePrediction.data.angularRateError/(statePrediction.data.angularRateError + gyroCov);
            navigationData_.data.angularRate = statePrediction.data.angularRate + rateGain*(angularRateRaw.data-statePrediction.data.angularRate);
            
            //Calculate new state errors
            navigationData_.data.angularAccelerationError = gyroAccelCov;//(Vector<>(1)-accelGain)*statePrediction.data.angularAccelerationError;
            navigationData_.data.angularRateError = (Vector<>(1)-rateGain)*statePrediction.data.angularRateError;

            Serial.println(String(", Gyro: ") + navigationData_.data.angularRate.x + ", " + navigationData_.data.angularRate.y + ", " + navigationData_.data.angularRate.z);

            if (!(rateGain.z == rateGain.z)) while(1);

            //Update old values
            lastAngularRateValue_ = angularRateRaw.data;
            lastAngularRateCov_ = gyroCov;
            lastGyroTimestamp_ = angularRateRaw.timestamp;

        }


        //Correct state prediction with accel values
        if (accelRaw.timestamp != 0 && accelRaw.timestamp == smallestTime) {
            
            //Retreive gyro data
            //accelSub_.removeBack();

            //Get delta time from last gyro data
            float dTime = float(accelRaw.timestamp - lastAccelTimestamp_)/SECONDS;

            //update buffers
            /*accelXBuffer_.placeFront(accelRaw.x, true);
            accelYBuffer_.placeFront(accelRaw.y, true);
            accelZBuffer_.placeFront(accelRaw.z, true);*/

            //Find out accel covariance and value in world coordinates
            accelRaw.data = statePrediction.data.attitude.rotateVector(accelRaw.data);
            //Vector<> accelMean = (lastAccelValue_ + accelRaw.data)*0.5;
            Vector<> accelCov = (accelRaw.data-statePrediction.data.acceleration).square();

            accelCovBuf_.placeFront(accelCov, true);
            accelCov = 0;
            for (uint32_t i = 0; i < accelCovBuf_.available(); i++) {
                accelCov += accelCovBuf_[i];
            }
            accelCov = sqrt(accelCov/accelCovBuf_.available());

            //Serial.println(String("Accel: ") + accelCov.x + ", " + accelCov.y + ", " + accelCov.z);

            //Fusion below for attitude is still a complementary filter.
            float beta = 0.1f; 

            //Z-Axis correction
            Vector<> zAxisIs = Vector<>(0,0,1);
            Vector<> zAxisSet = statePrediction.data.attitude.rotateVector(accelRaw.data);

            Vector<> zAxisRotationAxis = zAxisSet.cross(zAxisIs);
            float zAxisRotationAngle = zAxisSet.getAngleTo(zAxisIs);

            Quaternion<> zAxisCorrectionQuat = Quaternion<>(zAxisRotationAxis, zAxisRotationAngle*beta*dTime);


            //Apply state correction and normalise attitude quaternion 
            //navigationData_.data.attitude = zAxisCorrectionQuat*statePrediction.data.attitude;
            navigationData_.data.attitude.normalize(true);


            //Run kalman filter correction stuff 
            Vector<> accelGain = statePrediction.data.accelerationError/(statePrediction.data.accelerationError + accelCov);
            navigationData_.data.acceleration = statePrediction.data.acceleration + accelGain*(accelRaw.data-statePrediction.data.acceleration);

            navigationData_.data.linearAcceleration = navigationData_.data.acceleration + GRAVITY_VECTOR;
            
            //Calculate new state errors
            navigationData_.data.accelerationError = (Vector<>(1)-accelGain)*statePrediction.data.accelerationError;

            /*Serial.print(String("Pred: ") + statePrediction.data.acceleration.x + ", " + statePrediction.data.acceleration.y + ", " + statePrediction.data.acceleration.z);
            Serial.print(String(", Meas: ") + accelRaw.data.x + ", " + accelRaw.data.y + ", " + accelRaw.data.z);
            Serial.println();*/

            //if (!(accelGain.z == accelGain.z)) while(1);


            //Update old values
            lastAccelValue_ = accelRaw.data;
            lastAccelCov_ = accelCov;
            lastAccelTimestamp_ = accelRaw.timestamp;
            

        }

        //Serial.println(String("Time: ") + micros() + ", smallest: " + uint32_t(smallestTime));

        //Update for next iteration
        navigationData_.timestamp = smallestTime;

    }

    navigationDataTopic_.publish(navigationData_);

}



void NavigationKalman::setGyroscopeInput(const Gyroscope_Interface& gyro) {
    gyroSub_.subscribe(gyro.getGyroTopic());
}



void NavigationKalman::removeGyroscopeInput() {
    gyroSub_.unsubcribe();
}



void NavigationKalman::setAccelerometerInput(const Accelerometer_Interface& accel) {
    accelSub_.subscribe(accel.getAccelTopic());
}



void NavigationKalman::removeAccelerometerInput() {
    accelSub_.unsubcribe();
}



void NavigationKalman::setMagnetometerInput(const Magnetometer_Interface& mag) {
    magSub_.subscribe(mag.getMagTopic());
}



void NavigationKalman::removeMagnetometerInput() {
    magSub_.unsubcribe();
}



void NavigationKalman::setBarometerInput(const Barometer_Interface& baro) {
    baroSub_.subscribe(baro.getBaroTopic());
}



void NavigationKalman::removeBarometerInput() {
    baroSub_.unsubcribe();
}



void NavigationKalman::setGNSSInput(const GNSS_Interface& gnss) {
    gnssSub_.subscribe(gnss.getGNSSTopic());
}



void NavigationKalman::removeGNSSInput() {
    gnssSub_.unsubcribe();
}



void NavigationKalman::init() {

    navigationData_.data.angularAccelerationError = 10000;
    navigationData_.data.angularRateError = 10000;
    navigationData_.data.attitudeError = 10000;

    navigationData_.data.accelerationError = 10000;
    navigationData_.data.velocityError = 10000;
    navigationData_.data.positionError = 10000;

}
