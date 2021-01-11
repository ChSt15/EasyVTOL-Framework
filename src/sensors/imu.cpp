#include "imu.h"


namespace IMU {

    const int imuNCS = 3;
    const int imuInt = 2;
    //const int imuNCS = 33;
    //const int imuInt = 32;


    CircularBuffer <Vector, 100> gyroFifo;
    CircularBuffer <Vector, 100> accelFifo;
    CircularBuffer <Vector, 100> magFifo;

    Vector lastGyro;
    Vector lastAccel;
    Vector lastMag;

    IntervalControl imuInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    MPU9250 imu(SPI, imuNCS);

    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;
    uint32_t sensorRate = 0;
    uint32_t sensorCounter = 0;

    uint32_t lastMeasurement = 0;

    bool newDataInterrupt = false;

    void interruptRoutine();

}


void IMU::interruptRoutine() {
    newDataInterrupt = true;
}


void IMU::deviceThread() {

    if (!imuInterval.isTimeToRun()) return; 

    loopCounter++;


    if (imuStatus == DeviceStatus::DEVICE_RUNNING) {

        if (newDataInterrupt || digitalRead(imuInt)) { //If high then data is ready in the imu FIFO
            newDataInterrupt = false;

            sensorCounter++;

            imu.readSensor();

            Vector bufVec(imu.getGyroX_rads(), imu.getGyroY_rads(), imu.getGyroZ_rads());
            gyroFifo.unshift(bufVec);

            /*if (imu.readFifo()) { // read data and check if successful

                float gyroX[100];
                float gyroY[100];
                float gyroZ[100];

                float accelX[100];
                float accelY[100];
                float accelZ[100];

                float magX[100];
                float magY[100];
                float magZ[100];

                size_t bufferSize;

                imu.getFifoGyroX_rads(&bufferSize, gyroX);
                imu.getFifoGyroY_rads(&bufferSize, gyroY);
                imu.getFifoGyroZ_rads(&bufferSize, gyroZ);

                imu.getFifoAccelX_mss(&bufferSize, accelX);
                imu.getFifoAccelY_mss(&bufferSize, accelY);
                imu.getFifoAccelZ_mss(&bufferSize, accelZ);

                imu.getFifoMagX_uT(&bufferSize, magX);
                imu.getFifoMagY_uT(&bufferSize, magY);
                imu.getFifoMagZ_uT(&bufferSize, magZ);

                for (byte i = 0; i < bufferSize; i++) {

                    Vector bufVec(gyroX[i], gyroY[i], gyroZ[i]);
                    if (bufVec != lastGyro && !gyroFifo.isFull()) gyroFifo.unshift(bufVec);
                    lastGyro = bufVec;
                    Serial.println("Gyro X: " + String(lastGyro.x));

                    bufVec = Vector(accelX[i], accelY[i], accelZ[i]);
                    if (bufVec != lastAccel && !accelFifo.isFull()) accelFifo.unshift(bufVec);
                    lastAccel = bufVec;

                    bufVec = Vector(magX[i], magY[i], magZ[i]);
                    if (bufVec != lastMag && !magFifo.isFull()) magFifo.unshift(bufVec);
                    lastMag = bufVec;

                }

                lastMeasurement = micros();


            } else if (micros() - lastMeasurement >= SENSOR_MEASUREMENT_TIMEOUT_US) imuStatus = DeviceStatus::DEVICE_FAILURE;*/

        }

    } else if (imuStatus == DeviceStatus::DEVICE_NOT_STARTED || imuStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {
        

        int startCode = imu.begin();


        if (startCode > 0) {

            imuInterval.setRate(8000);

            imu.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G); //Yes this and the gyro range being so high will make it less accurate but we also dont want to loose information. This could be changed later depending on the application
            imu.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
            //imu.enableFifo(true, true, true, false); //We do not need the temperature so it will be disabled
            imu.enableDataReadyInterrupt();

            //###################### Following will be changed in the future to allow higher rates #####################
            imu.setSrd(0);
            imu.setDlpfBandwidth(MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_184HZ);

            attachInterrupt(imuInt, interruptRoutine, HIGH);

            lastMeasurement = micros();

            imuStatus = DeviceStatus::DEVICE_RUNNING;

        } else {
            imuStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 
            Serial.println("IMU Start Fail. Code: " + String(startCode));
        }

        startAttempts++;

        if (startAttempts >= 5 && imuStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) imuStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (imuStatus == DeviceStatus::DEVICE_CALIBRATING) {

        //################## Following is Temporary #################
        if (imu.calibrateGyro()) imuStatus = DeviceStatus::DEVICE_RUNNING; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        imuStatus = DeviceStatus::DEVICE_FAILURE;
        imuInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        sensorRate = sensorCounter;
        sensorCounter = 0;
        loopCounter = 0;
    }

}


uint16_t IMU::getMeasurementRate() {
    return sensorRate;
}


uint32_t IMU::getRate() {
    return rate;
}


DeviceStatus IMU::getDeviceStatus() {return imuStatus;}


