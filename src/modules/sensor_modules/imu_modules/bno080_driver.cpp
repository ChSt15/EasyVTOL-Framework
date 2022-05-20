#include "KraftKontrol/modules/sensor_modules/imu_modules/bno080_driver.h"



volatile int64_t BNO080Driver::_newDataTimestamp = 0;
volatile bool BNO080Driver::_newDataInterrupt = false;
Task_Threading* BNO080Driver::driverTask_ = nullptr;



BNO080Driver::BNO080Driver(int interruptPin, TwoWire& i2cBus, const Barometer_Abstract* baro, const GNSS_Abstract* gnss): Task_Threading("BNO080 Driver", eTaskPriority_t::eTaskPriority_VeryHigh, SECONDS/200), pinInterrupt_(interruptPin, _interruptRoutine, false, true), i2c_(i2cBus) {
    if (baro != nullptr) baroSubr_.subscribe(baro->getBaroTopic());
    if (gnss != nullptr) gnssSubr_.subscribe(gnss->getGNSSTopic());
}


void BNO080Driver::prediction() {

    
    float dTime = (float)(NOW() - navigationData_.timestamp)/SECONDS;

    //Predict current state and its error
    navigationData_.data.velocity += navigationData_.data.linearAcceleration*dTime;
    navigationData_.data.velocityError += navigationData_.data.accelerationError*dTime;

    navigationData_.data.position += navigationData_.data.velocity*dTime;
    navigationData_.data.absolutePosition.height = navigationData_.data.position.z + navigationData_.data.homePosition.height;
    navigationData_.data.positionError += navigationData_.data.velocityError*dTime;
    
    
}


void BNO080Driver::getGNSSData() {

    const DataTimestamped<GNSSData>& data = gnssSubr_.getItem();

    ValueError<Vector<>> posBuf = ValueError<Vector<>>(data.data.position.getPositionVectorFrom(navigationData_.data.homePosition), data.data.positionError);
    ValueError<Vector<>> velBuf = ValueError<Vector<>>(data.data.velocity, data.data.velocityError);

    posBuf = ValueError<Vector<>>(navigationData_.data.position, navigationData_.data.positionError).weightedAverage(posBuf);
    velBuf = ValueError<Vector<>>(navigationData_.data.velocity, navigationData_.data.velocityError).weightedAverage(velBuf);

    navigationData_.data.position = posBuf.value;
    //navigationData_.data.position.y = posBuf.value.y;
    //navigationData_.data.position.z = posBuf.value.z;

    navigationData_.data.positionError = posBuf.error;
    //navigationData_.data.positionError.y = posBuf.error.y;
    //navigationData_.data.positionError.z = posBuf.error.z;

    navigationData_.data.velocity = velBuf.value;
    //navigationData_.data.velocity.y = velBuf.value.y;
    //navigationData_.data.velocity.z = velBuf.value.z;

    navigationData_.data.velocityError = velBuf.error;
    //navigationData_.velocityError.y = velBuf.error.y;
    //navigationData_.velocityError.z = velBuf.error.z;

    

}


void BNO080Driver::getBaroData() {

    /*SensorTimestamp<float> sensorTime;

    do {    

        baroSub_.takeBack(sensorTime);

        float& baroPressure_ = sensorTime.data;
        int64_t& timestamp = sensorTime.sensorTimestamp;

        //calculate height from new pressure value
        float heightAbsolute = getHeightFromPressure(baroPressure_, sealevelPressure_);
        //Serial.println(String("H: ") + heightAbsolute);
        //float heightRelative = heightAbsolute - navigationData_.absolutePosition.height;
        //calculate z velocity from new height value
        float zVelocity = (heightAbsolute - _lastHeightValue)/dt;
        _lastHeightValue = heightAbsolute;

        //Update buffers
        baroHeightBuf_.placeFront(heightAbsolute, true);
        baroVelBuf_.placeFront(zVelocity, true);

        float heightMedian = baroHeightBuf_.getMedian();
        float heightError = baroHeightBuf_.getStandardError();

        float velMedian = baroVelBuf_.getMedian();
        float velError = baroVelBuf_.getStandardError();

        //Serial.println(String("Height: ") + heightMedian + " +- " + String(heightError,5) + ",\tvel: " + velMedian + "+-" + velError);
        //Serial.println(String("") + (heightMedian - navigationData_.homePosition.height) + "  " + velMedian);

        //correct dead reckoning values with new ones.
        ValueError<> heightVel = ValueError<>(navigationData_.velocity.z, navigationData_.velocityError.z).weightedAverage(ValueError<>(zVelocity, velError));
        navigationData_.velocity.z = heightVel.value;
        navigationData_.velocityError.z = heightVel.error;

        ValueError<> height = ValueError<>(navigationData_.absolutePosition.height, navigationData_.positionError.z).weightedAverage(ValueError<>(heightAbsolute, heightError));
        navigationData_.absolutePosition.height = height.value;
        navigationData_.positionError.z = height.error;

        //Update position z
        navigationData_.position.z = navigationData_.absolutePosition.height - navigationData_.homePosition.height;
    
    } while(baroSubr_.available() > 0);*/

}


void BNO080Driver::getIMUData() {

    //i2c_.setClock(400000);
    _imu.getReadings();

    uint8_t accuracy;

    float angleAccuracy;

    FML::Quaternion<> transform = FML::Quaternion<>(Vector<>(0,0,1), -90*DEG_TO_RAD)*FML::Quaternion<>(Vector<>(0,0,1), 90*DEG_TO_RAD)*FML::Quaternion<>(Vector<>(1,0,0), 180*DEG_TO_RAD);

    DataTimestamped<FML::Quaternion<>> quat(0, _newDataTimestamp);
    _imu.getQuat(quat.data.x, quat.data.y, quat.data.z, quat.data.w, angleAccuracy, accuracy);
    navigationData_.data.attitude = quat.data*transform;

    DataTimestamped<Vector<>> bufVec(0, _newDataTimestamp);
    _imu.getGyro(bufVec.data.x, bufVec.data.y, bufVec.data.z, accuracy);
    navigationData_.data.angularRate = navigationData_.data.attitude.rotateVector(bufVec.data);

    _imu.getAccel(bufVec.data.x, bufVec.data.y, bufVec.data.z, accuracy);
    navigationData_.data.acceleration = navigationData_.data.attitude.rotateVector(bufVec.data);

    _imu.getLinAccel(bufVec.data.x, bufVec.data.y, bufVec.data.z, accuracy);
    navigationData_.data.linearAcceleration = navigationData_.data.attitude.rotateVector(bufVec.data);
    

}


void BNO080Driver::thread() {


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        prediction();

        if (_newDataInterrupt) { //If true then data is ready in the imu FIFO
            _newDataInterrupt = false;

            getIMUData();
            

        }

        if (baroSubr_.available() > 0) getBaroData();

        if (gnssSubr_.isDataNew()) getGNSSData();
        
        navigationData_.timestamp = NOW();

        navigationDataTopic_.publish(navigationData_);


    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        Serial.println("CALIBRATING IMU");
        //imu.calibrateGyro();
        //imu.calibrateMag();
        for (byte n = 0; n < 6; n++) {
            if (n != 0) Serial.println("SWITCH");
            //_imu.calibrateAccel();
        }

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        driverTask_ = nullptr;
        suspendUntil(END_OF_TIME);

    }

}



void BNO080Driver::_interruptRoutine() {
    _newDataInterrupt = true;
    _newDataTimestamp =  NOW();
    //if (driverTask_ != nullptr) driverTask_->startTaskThreading();
}



void BNO080Driver::init() {

    i2c_.begin();
    i2c_.setClock(100000);

    //Serial.println("Test1");
    bool startCode = _imu.begin(BNO080_DEFAULT_ADDRESS, i2c_);
    //Serial.println("Test2");

    i2c_.setClock(400000);


    if (startCode) {

        _imu.enableAccelerometer(5);
        _imu.enableGyro(5);
        _imu.enableGameRotationVector(5);

        //pinInterrupt_.setEnable(true);
        
        //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        driverTask_ = this;

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("IMU Start Fail. Code: " + String(startCode));
    }

    startAttempts_++;

    if (startAttempts_ >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
