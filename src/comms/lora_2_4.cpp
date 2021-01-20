#include "lora_2_4.h"


namespace LORA_2_4 {

    const int NSSPin = SX1280_NSS_PIN;
    const int NRESETPin = SX1280_NRESET_PIN;
    const int RFBUSYPin = SX1280_RFBUSY_PIN;
    const int DIO1Pin = SX1280_DIO1_PIN;
    const int RXENPin = SX1280_RXEN_PIN;
    const int TXENPin = SX1280_TXEN_PIN;


    CircularBuffer <Quaternion, 100> attitudeSendFifo;


    IntervalControl loraInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    SX128XLT lora;

    DeviceStatus loraStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;
    uint32_t sensorRate = 0;
    uint32_t sensorCounter = 0;

    bool newDataInterrupt = false;

    void interruptRoutine();

}


void LORA_2_4::interruptRoutine() {
    newDataInterrupt = true;
}


void LORA_2_4::deviceThread() {

    if (!loraInterval.isTimeToRun()) return; 

    loopCounter++;


    if (loraStatus == DeviceStatus::DEVICE_RUNNING) {

        if (newDataInterrupt) { //If high then data is ready in the lora FIFO
            newDataInterrupt = false;

            sensorCounter++;


            

        }

    } else if (loraStatus == DeviceStatus::DEVICE_NOT_STARTED || loraStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {
        
        if (loraStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) lora.resetDevice();
        
        if (lora.begin(NSSPin, NRESETPin, RFBUSYPin, DIO1Pin, RXENPin, TXENPin, DEVICE_SX1280)) {

            loraInterval.setRate(COMMS_RATE);

            lora.setupLoRa(LORA_FREQUENCY, 0, LORA_SPREADINGFACTOR, LORA_BANDWIDTH, LORA_CODERATE);

            lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);

            attachInterrupt(DIO1Pin, interruptRoutine, HIGH);

            loraStatus = DeviceStatus::DEVICE_RUNNING;

        } else {
            loraStatus = DeviceStatus::DEVICE_RESTARTATTEMPT; 
        }

        startAttempts++;

        if (startAttempts >= 5 && loraStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) loraStatus = DeviceStatus::DEVICE_FAILURE;

    } else if (loraStatus == DeviceStatus::DEVICE_CALIBRATING) {

        

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        loraStatus = DeviceStatus::DEVICE_FAILURE;
        loraInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        sensorRate = sensorCounter;
        sensorCounter = 0;
        loopCounter = 0;
    }

}


uint32_t LORA_2_4::getRate() {
    return rate;
}


DeviceStatus LORA_2_4::getDeviceStatus() {return loraStatus;}


