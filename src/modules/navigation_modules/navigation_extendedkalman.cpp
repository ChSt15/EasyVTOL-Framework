#include "KraftKontrol/modules/navigation_modules/navigation_extendedkalman.h"



NavigationExtendedKalman::NavigationExtendedKalman() : Task_Abstract("Nav Extended Kalmanfilter", 20, eTaskPriority_t::eTaskPriority_High) {}



void NavigationExtendedKalman::thread() {

    //Gather valid sensor values
    if (gyroSub_.isDataNew()) {
        gyroData_ = gyroSub_.getItem();
    }

    if (accelSub_.isDataNew()) {
        accelData_ = accelSub_.getItem();
    }

    if (magSub_.isDataNew()) {
        magData_ = magSub_.getItem();
    }

    if (baroSub_.isDataNew()) {
        baroData_ = baroSub_.getItem();
    }

    if (gnssSub_.isDataNew()) {
        gnssData_ = gnssSub_.getItem();
    }



    if (gyroData_.timestamp != lastGyroData_.timestamp) {   //Gyro data is new. Do an update.

        float dt = float(gyroData_.timestamp - navigationData_.timestamp)/SECONDS;


        

        

    }


    if (accelData_.timestamp != lastAccelData_.timestamp) {   //Accel data is new. Do an update to extended kalman filter.

        

    }


    if (magData_.timestamp != lastMagData_.timestamp) {   //mag data is new. Do an update.

        

    }



    //Move calculated data to old data container type

    //navigationData_.data.attitude.w = X(0);     navigationData_.data.attitude.x = X(1);     navigationData_.data.attitude.y = X(2);     navigationData_.data.attitude.z = X(3);
    //navigationData_.data.angularRate.x = X(4);  navigationData_.data.angularRate.y = X(5);  navigationData_.data.angularRate.z = X(6);


    lastGyroData_ = gyroData_;
    lastAccelData_ = accelData_;
    lastMagData_ = magData_;
    lastBaroData_ = baroData_;
    lastGnssData_ = gnssData_;
    
    navigationData_.timestamp = NOW();
    navigationDataTopic_.publish(navigationData_);


}



void NavigationExtendedKalman::setGyroscopeInput(const Gyroscope_Abstract& gyro) {
    gyroSub_.subscribe(gyro.getGyroTopic());
}



void NavigationExtendedKalman::removeGyroscopeInput() {
    gyroSub_.unsubcribe();
}



void NavigationExtendedKalman::setAccelerometerInput(const Accelerometer_Abstract& accel) {
    accelSub_.subscribe(accel.getAccelTopic());
}



void NavigationExtendedKalman::removeAccelerometerInput() {
    accelSub_.unsubcribe();
}



void NavigationExtendedKalman::setMagnetometerInput(const Magnetometer_Abstract& mag) {
    magSub_.subscribe(mag.getMagTopic());
}



void NavigationExtendedKalman::removeMagnetometerInput() {
    magSub_.unsubcribe();
}



void NavigationExtendedKalman::setBarometerInput(const Barometer_Abstract& baro) {
    baroSub_.subscribe(baro.getBaroTopic());
}



void NavigationExtendedKalman::removeBarometerInput() {
    baroSub_.unsubcribe();
}



void NavigationExtendedKalman::setGNSSInput(const GNSS_Abstract& gnss) {
    gnssSub_.subscribe(gnss.getGNSSTopic());
}



void NavigationExtendedKalman::removeGNSSInput() {
    gnssSub_.unsubcribe();
}



void NavigationExtendedKalman::init() {

    navigationData_.data.angularAccelerationError = 10000000;
    navigationData_.data.angularRateError = 10000000;
    navigationData_.data.attitudeError = 10000000;

    navigationData_.data.accelerationError = 10000000;
    navigationData_.data.velocityError = 10000000;
    navigationData_.data.positionError = 10000000;

}
