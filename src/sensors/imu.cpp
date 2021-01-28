#include "imu.h"


namespace IMU {

    const int imuNCS = MPU_NCS_PIN;
    const int imuInt = MPU_INT_PIN;


    CircularBuffer <Vector, 100> gyroFifo;
    CircularBuffer <Vector, 100> accelFifo;
    CircularBuffer <Vector, 100> magFifo;
    CircularBuffer <uint32_t, 100> gyroTimestampFifo;
    CircularBuffer <uint32_t, 100> accelTimestampFifo;
    CircularBuffer <uint32_t, 100> magTimestampFifo;

    Vector lastGyro;
    Vector lastAccel;
    Vector lastMag;

    IntervalControl imuInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    MPU9250 imu(SPI, imuNCS);

    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t loopRate = 0;
    uint32_t loopCounter = 0;

    uint32_t gyroRate = 0;
    uint32_t gyroCounter = 0;

    uint32_t accelRate = 0;
    uint32_t accelCounter = 0;

    uint32_t magRate = 0;
    uint32_t magCounter = 0;

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

        if (newDataInterrupt) { //If true then data is ready in the imu FIFO
            newDataInterrupt = false;


            imu.readSensor();
            uint32_t timestamp = micros();

            Vector bufVec(-imu.getGyroX_rads(), -imu.getGyroY_rads(), imu.getGyroZ_rads());
            if (lastGyro != bufVec) {
                gyroFifo.unshift(bufVec);
                gyroTimestampFifo.unshift(timestamp);
                lastGyro = bufVec;
                gyroCounter++;
            }

            bufVec = Vector(imu.getAccelX_mss(), imu.getAccelY_mss(), -imu.getAccelZ_mss());
            if (lastAccel != bufVec) {
                //bufVec = bufVec*0.001f + lastAccel*0.999f;
                accelFifo.unshift(bufVec);
                accelTimestampFifo.unshift(timestamp);
                lastAccel = bufVec;
                accelCounter++;
            }

            bufVec = Vector(-imu.getMagX_uT(), -imu.getMagY_uT(), imu.getMagZ_uT());
            if (lastMag != bufVec) {
                magFifo.unshift(bufVec);
                magTimestampFifo.unshift(timestamp);
                lastMag = bufVec;
                magCounter++;
            }

            //Vector testVec = gyroFifo.first();
            //Vector bias = Vector(imu.getAccelBiasX_mss(), imu.getAccelBiasY_mss(), imu.getAccelBiasZ_mss());
            //Vector scale = Vector(imu.getAccelScaleFactorX(), imu.getAccelScaleFactorY(), imu.getAccelScaleFactorZ());

            //Serial.println("AccelBias: x: " + String(bias.x) + ", y: " + String(bias.y) + ", z: " + String(bias.z));
            //Serial.println("AccelScale: x: " + String(scale.x) + ", y: " + String(scale.y) + ", z: " + String(scale.z));

            //Serial.println("MagBias: x: " + String(MagCal.x) + ", y: " + String(MagCal.y) + ", z: " + String(MagCal.z));
            //Serial.println("Mag: x: " + String(lastMag.x) + ", y: " + String(lastMag.y) + ", z: " + String(lastMag.z));
            //Serial.println("Gyro: x: " + String(testVec.x) + ", y: " + String(testVec.y) + ", z: " + String(testVec.z));
            //Serial.println("Accel: x: " + String(lastAccel.x,5) + ", y: " + String(lastAccel.y,5) + ", z: " + String(lastAccel.z,5));
            //Serial.println();


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

                    Vector bufVec(-gyroX[i], -gyroY[i], gyroZ[i]);
                    if (lastGyro != bufVec) {
                        static uint32_t timestamp = micros();
                        gyroFifo.unshift(bufVec);
                        gyroTimestampFifo.unshift(timestamp);
                        timestamp += 1000000/gyroRate;
                        lastGyro = bufVec;
                        gyroCounter++;
                    }

                    bufVec = Vector(accelX[i], accelY[i], -accelZ[i]);
                    if (lastAccel != bufVec) {
                        static uint32_t timestamp = micros();
                        accelFifo.unshift(bufVec);
                        accelTimestampFifo.unshift(timestamp);
                        timestamp += 1000000/accelRate;
                        lastAccel = bufVec;
                        accelCounter++;
                    }

                    bufVec = Vector(-magX[i], -magY[i], magZ[i]);
                    if (lastMag != bufVec) {
                        static uint32_t timestamp = micros();
                        magFifo.unshift(bufVec);
                        magTimestampFifo.unshift(timestamp);
                        timestamp += 1000000/magRate;
                        lastMag = bufVec;
                        magCounter++;
                    }

                }

                lastMeasurement = micros();


            } else if (micros() - lastMeasurement >= SENSOR_MEASUREMENT_TIMEOUT_US) imuStatus = DeviceStatus::DEVICE_FAILURE;*/

        }

    } else if (imuStatus == DeviceStatus::DEVICE_NOT_STARTED || imuStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {
        
        Serial.println("Test1");
        int startCode = imu.begin();
        Serial.println("Test2");


        if (startCode > 0) {

            imuInterval.setRate(IMU_RATE_LIMIT);

            imu.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G); //Yes this and the gyro range being so high will make it less accurate but we also dont want to loose information. This could be changed later depending on the application
            imu.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
            //imu.enableFifo(true, true, true, false); //We do not need the temperature so it will be disabled
            imu.enableDataReadyInterrupt();

            //###################### Following will be changed in the future to allow higher rates #####################
            imu.setSrd(0);
            imu.setDlpfBandwidth(MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_184HZ);

            imu.setMagCalX(18.26, 1.0f);
            imu.setMagCalY(39.14, 1.0f);
            imu.setMagCalZ(-40.69, 1.0f);

            imu.setAccelCalX(0.20268, 1.0f);
            imu.setAccelCalY(-0.14238, 1.001f);
            imu.setAccelCalZ(0.6755, 0.9869f);

            attachInterrupt(imuInt, interruptRoutine, HIGH);

            lastMeasurement = micros();
            

            //imuStatus = DeviceStatus::DEVICE_CALIBRATING;
            imuStatus = DeviceStatus::DEVICE_RUNNING;

        } else {
            imuStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 
            Serial.println("IMU Start Fail. Code: " + String(startCode));
        }

        startAttempts++;

        if (startAttempts >= 5 && imuStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) imuStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (imuStatus == DeviceStatus::DEVICE_CALIBRATING) {

        //################## Following is Temporary #################
        Serial.println("CALIBRATING IMU");
        //imu.calibrateGyro();
        //imu.calibrateMag();
        for (byte n = 0; n < 6; n++) {
            if (n != 0) Serial.println("SWITCH");
            imu.calibrateAccel();
        }

        imuStatus = DeviceStatus::DEVICE_RUNNING; 
        //else imuStatus = DeviceStatus::DEVICE_FAILURE; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        imuStatus = DeviceStatus::DEVICE_FAILURE;
        imuInterval.block(true);
        loopRate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        loopRate = loopCounter;
        gyroRate = gyroCounter;
        accelRate = accelCounter;
        magRate = magCounter;
        gyroCounter = 0;
        accelCounter = 0;
        magCounter = 0;
        loopCounter = 0;
    }

}


uint32_t IMU::getGyroRate() {
    return gyroRate;
}

uint32_t IMU::getAccelRate() {
    return accelRate;
}

uint32_t IMU::getMagRate() {
    return magRate;
}

uint32_t IMU::getLoopRate() {
    return loopRate;
}


DeviceStatus IMU::getDeviceStatus() {return imuStatus;}


bool IMU::gyroAvailable() {return !gyroFifo.isEmpty();};
bool IMU::getGyro(Vector* gyro, uint32_t* timestamp) {
    if (gyroFifo.isEmpty()) return false;
    *gyro = gyroFifo.pop();
    *timestamp = gyroTimestampFifo.pop();
    return true;
}

bool IMU::accelAvailable() {return !accelFifo.isEmpty();};
bool IMU::getAccel(Vector* accel, uint32_t* timestamp) {
    if (accelFifo.isEmpty()) return false;
    *accel = accelFifo.pop();
    *timestamp = accelTimestampFifo.pop();
    return true;
}

bool IMU::magAvailable() {return !magFifo.isEmpty();};
bool IMU::getMag(Vector* mag, uint32_t* timestamp) {
    if (magFifo.isEmpty()) return false;
    *mag = magFifo.pop();
    *timestamp = magTimestampFifo.pop();
    return true;
}


