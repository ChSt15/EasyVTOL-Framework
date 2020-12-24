#include "sensor_template.h"



namespace SensorTemplate {

    /*
     * Here are where all variables must be declared.
     * If they are to be seen from outside (public) then declare
     * them AGAIN in the .h file namespace with extern as a
     * keyword at the beginning of declaration.
     * See sensorDataFifo as an example.
     * If the variables are to be private then only declare them 
     * here.
    */

    CircularBuffer <float, 50> sensorDataFifo; //All new sensor data is stored here
    CircularBuffer <uint32_t, 50> timestampFifo; //Timstamp storage is stored here

    IntervalControl sensorInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    DeviceStatus sensorStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    uint32_t lastMeasurement = 0;

}


void SensorTemplate::sensorThread() { //This methode public because it was declares in the imu.h files namespace

    if (!sensorInterval.isTimeToRun()) return; //This limits the rate the thread will run at to save time. By default the rate is infinite until the code below resticts it (pls do this).

    loopCounter++; //Counter for loop speed


    if (sensorStatus == DeviceStatus::DEVICE_RUNNING) {

        if (/*check id sensor has new data. True SHOULD be removed*/ true) {

            //Here code for handling the sensor data.

        } else if (micros() - lastMeasurement >= SENSOR_MEASUREMENT_TIMEOUT_US) sensorStatus = DeviceStatus::DEVICE_FAILURE; // Check for sensor timeout. This is a safeguard

    } else if (sensorStatus == DeviceStatus::DEVICE_NOT_STARTED || sensorStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        if (/*Try to start sensor and check if start was successfull. True SHOULD be removed*/ true) {

            //Code to setup the sensor...

            sensorInterval.setRate(1000); // Set the maximum rate the device loop should run at. Here 1kHz.
            lastMeasurement = micros();
            sensorStatus = DeviceStatus::DEVICE_RUNNING; //Skip calibration, 

        } else sensorStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 

        startAttempts++; //Count how often sensor start has occured

        if (startAttempts >= 5 && sensorStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) sensorStatus = DeviceStatus::DEVICE_FAILURE; //If 5 attempts were made then the sensor will not work

    } else if (sensorStatus == DeviceStatus::DEVICE_CALIBRATING) {

        /*
         * if sensorStatus was set to DEVICE_CALIBRATING then this will run after starting the sensor.
        */

       sensorStatus = DeviceStatus::DEVICE_RUNNING; //Go to normal working mode

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure and dont allow the loop to continue.

        sensorStatus = DeviceStatus::DEVICE_FAILURE;
        sensorInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) { //This can be left alone. It only measures how fast the loop if running
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint32_t SensorTemplate::getRate() { //This returns how fast the loop is running
    return rate;
}


DeviceStatus SensorTemplate::getDeviceStatus() {return sensorStatus;} //Returns the device status





