#include "ibus_rx.h"


namespace IBUSReceiver {


    CircularBuffer <float[16], 100> channelData;


    IntervalControl rxInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    DeviceStatus rxStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;
    uint32_t sensorRate = 0;
    uint32_t sensorCounter = 0;

    bool newDataInterrupt = false;

}



void IBUSReceiver::deviceThread() {

    if (!rxInterval.isTimeToRun()) return; 

    loopCounter++;


    if (rxStatus == DeviceStatus::DEVICE_RUNNING) {

        if (true) { //If high then data is ready

            

        }

    } else if (rxStatus == DeviceStatus::DEVICE_NOT_STARTED || rxStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {
        
        
        if (true) {

            

            rxStatus = DeviceStatus::DEVICE_RUNNING;

        } else {
            rxStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 
        }

        startAttempts++;

        if (startAttempts >= 5 && rxStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) rxStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (rxStatus == DeviceStatus::DEVICE_CALIBRATING) {

        

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        rxStatus = DeviceStatus::DEVICE_FAILURE;
        rxInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        sensorRate = sensorCounter;
        sensorCounter = 0;
        loopCounter = 0;
    }

}


uint32_t IBUSReceiver::getRate() {
    return rate;
}


DeviceStatus IBUSReceiver::getDeviceStatus() {return rxStatus;}


