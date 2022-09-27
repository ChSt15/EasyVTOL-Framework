#include "KraftKontrol/modules/navigation_modules/navigation_kalman_unav.h"



NavigationKalmanUNAV::NavigationKalmanUNAV(SystemModel_Abstract& systemModel) : Task_Abstract("Nav Kalmanfilter", 50, eTaskPriority_t::eTaskPriority_High), systemModel_(systemModel) {}



void NavigationKalmanUNAV::thread() {

    //Gather valid sensor values
    gyroData.timestamp = 0;
    if (gyroSub_.isDataNew()) {
        firstGyroWait = true;
        gyroData = gyroSub_.getItem();
    }

    accelData.timestamp = 0;
    if (accelSub_.isDataNew()) {
        firstAccelWait = true;
        accelData = accelSub_.getItem();
    }

    magData.timestamp = 0;
    if (magSub_.isDataNew()) {
        magData = magSub_.getItem();
    }

    DataTimestamped<float> baroRaw;
    if (baroSub_.isDataNew()) {
        baroRaw = baroSub_.getItem();
    }

    DataTimestamped<GNSSData> gnssRaw;
    if (gnssSub_.isDataNew()) {
        gnssRaw = gnssSub_.getItem();
    }


    if (firstGyroWait && firstAccelWait) {

        filter.update(gnssRaw.timestamp, gnssRaw.data.velocity.x, -gnssRaw.data.velocity.y, -gnssRaw.data.velocity.z, gnssRaw.data.position.latitude, gnssRaw.data.position.longitude, gnssRaw.data.position.height, gyroData.data.values(0), gyroData.data.values(1), gyroData.data.values(2), accelData.data.values(0), accelData.data.values(1), accelData.data.values(2), magData.data.values(0), magData.data.values(1), magData.data.values(2));

        navigationData_.data.attitude.w = filter.getQuatW();
        navigationData_.data.attitude.x = filter.getQuatX();
        navigationData_.data.attitude.y = filter.getQuatY();
        navigationData_.data.attitude.z = filter.getQuatZ();

        Vector<> gyroBody;
        gyroBody.x = gyroData.data.values(0) - filter.getGyroBiasX_rads();
        gyroBody.y = gyroData.data.values(1) - filter.getGyroBiasY_rads();
        gyroBody.z = gyroData.data.values(2) - filter.getGyroBiasZ_rads();
        navigationData_.data.angularRate = navigationData_.data.attitude.rotateVector(gyroBody);

        Vector<> accelBody;
        accelBody.x = accelData.data.values(0) - filter.getAccelBiasX_mss();
        accelBody.y = accelData.data.values(1) - filter.getAccelBiasY_mss();
        accelBody.z = accelData.data.values(2) - filter.getAccelBiasZ_mss();
        navigationData_.data.acceleration = navigationData_.data.attitude.rotateVector(accelBody);

        navigationData_.data.linearAcceleration = navigationData_.data.acceleration - GRAVITY_VECTOR;

        navigationData_.data.velocity.x = filter.getVelNorth_ms();
        navigationData_.data.velocity.y = -filter.getVelEast_ms();
        navigationData_.data.velocity.z = -filter.getVelDown_ms();

        navigationData_.data.absolutePosition.latitude = filter.getLatitude_rad();
        navigationData_.data.absolutePosition.longitude = filter.getLongitude_rad();
        navigationData_.data.absolutePosition.height = filter.getAltitude_m();

        navigationData_.data.position = navigationData_.data.absolutePosition.getPositionVectorFrom(navigationData_.data.homePosition);

        navigationData_.data.angularAcceleration = 0;

        navigationData_.timestamp = NOW();
    
        navigationDataTopic_.publish(navigationData_);

    }

}



void NavigationKalmanUNAV::setGyroscopeInput(const Gyroscope_Abstract& gyro) {
    gyroSub_.subscribe(gyro.getGyroTopic());
}



void NavigationKalmanUNAV::removeGyroscopeInput() {
    gyroSub_.unsubcribe();
}



void NavigationKalmanUNAV::setAccelerometerInput(const Accelerometer_Abstract& accel) {
    accelSub_.subscribe(accel.getAccelTopic());
}



void NavigationKalmanUNAV::removeAccelerometerInput() {
    accelSub_.unsubcribe();
}



void NavigationKalmanUNAV::setMagnetometerInput(const Magnetometer_Abstract& mag) {
    magSub_.subscribe(mag.getMagTopic());
}



void NavigationKalmanUNAV::removeMagnetometerInput() {
    magSub_.unsubcribe();
}



void NavigationKalmanUNAV::setBarometerInput(const Barometer_Abstract& baro) {
    baroSub_.subscribe(baro.getBaroTopic());
}



void NavigationKalmanUNAV::removeBarometerInput() {
    baroSub_.unsubcribe();
}



void NavigationKalmanUNAV::setGNSSInput(const GNSS_Abstract& gnss) {
    gnssSub_.subscribe(gnss.getGNSSTopic());
}



void NavigationKalmanUNAV::removeGNSSInput() {
    gnssSub_.unsubcribe();
}



void NavigationKalmanUNAV::init() {

    navigationData_.data.angularAccelerationError = 10000000;
    navigationData_.data.angularRateError = 10000000;
    navigationData_.data.attitudeError = 10000000;

    navigationData_.data.accelerationError = 10000000;
    navigationData_.data.velocityError = 10000000;
    navigationData_.data.positionError = 10000000;

}
