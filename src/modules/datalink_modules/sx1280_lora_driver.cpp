#include "sx1280_lora_driver.h"



void SX1280Driver::thread() {

    if (block_) return;

    loopCounter_++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        internalLoop();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {

        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        block_ = true;
        stopTaskThreading();
        loopRate_ = 0;

    }



    if (rateCalcInterval_.isTimeToRun()) {
        loopRate_ = loopCounter_;
        loopCounter_ = 0;
    }

}



void SX1280Driver::internalLoop() {


    if (digitalRead(dio1Pin_)) { //Something triggered interrupt

        #ifdef SX1280_DEBUG
            Serial.println("Radio triggered interrupt");
        #endif
        

        uint16_t irqStatus = radio_.readIrqStatus();


        if (irqStatus & (IRQ_RX_DONE + IRQ_HEADER_VALID)) { // If interrupt says data good then get data

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says data received");
            #endif

            byte packetL = radio_.readRXPacketL();

            if (packetL != 0) { // make sure packet is okay

                radio_.startReadSXBuffer(0);

                int i;
                for (i = 0; i < packetL && i < SX1280_DATA_BUFFER_SIZE; i++) receivedData_[i] = radio_.readUint8();

                radio_.endReadSXBuffer();

                receivedDataRSSI_ = radio_.readPacketRSSI();
                receivedDataSNR_ = radio_.readPacketSNR();

                receivedDataSize_ = packetL;

                #ifdef SX1280_DEBUG
                    Serial.println("Data Received and is " + String(packetL) + " bytes long. At RSSI: " + receivedDataRSSI_ + ", SNR: " + receivedDataSNR_);
                #endif

            } 
            #ifdef SX1280_DEBUG
                else {
                    Serial.println("Data was 0 bytes long! CRITICAL ERROR"); 
                    
                }
            #endif
        
        }
        
        if (irqStatus & (IRQ_RX_TIMEOUT)) {

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says rx timed out!");
            #endif

        } 
        
        if (irqStatus & (IRQ_TX_DONE)) {

            isBusySending_ = false;
            toSendDataSize_ = 0;

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says tx done!");
            #endif

        }

        if (irqStatus & (IRQ_TX_TIMEOUT)) {

            isBusySending_ = false;

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says tx timeout!");
            #endif

        }

        radio_.clearIrqStatus(IRQ_RADIO_ALL); 

        radio_.receive(receivedData_, SX1280_DATA_BUFFER_SIZE, 0, NO_WAIT);

    }


    
    if (toSendDataSize_ > 0 && !isBusySending_) { 

        isBusySending_ = true;

        radio_.transmit(toSendData_, toSendDataSize_, 0, SX1280_POWER_dB, NO_WAIT);

        #ifdef SX1280_DEBUG
            Serial.println("Sending data packet!");
        #endif

    }

}



void SX1280Driver::init() {

    SPI.begin();
    //SPI.setFrequency(10000000);

    if (radio_.begin(nssPin_, resetPin_, busyPin_, dio1Pin_, rxenPin_, txenPin_, DEVICE_SX1280)) {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        radio_.setupLoRa(SX1280_FREQUENCY, 0, SX1280_SPREADFACTOR, SX1280_BANDWIDTH, SX1280_CODINGRATE);
        radio_.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
        radio_.setHighSensitivity();

        radio_.receive(receivedData_, SX1280_DATA_BUFFER_SIZE, 0, NO_WAIT);

        Serial.println("SX1280 start success!");

        startAttempts_ = 0;

    } else {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 

        Serial.println("SX1280 start failure!");

    }

    startAttempts_++;

    if (startAttempts_ >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}



bool SX1280Driver::busy() {return isBusySending_;}



uint8_t SX1280Driver::sendBuffer(uint8_t* buffer, uint8_t size) {

    if (size > SX1280_DATA_BUFFER_SIZE) return 0;

    toSendDataSize_ = size;

    for (uint16_t i = 0; i < size; i++) toSendData_[i] = buffer[i];
    
    return size;

}



uint8_t SX1280Driver::available() {return receivedDataSize_;}



uint8_t SX1280Driver::receiveBuffer(uint8_t* buffer, uint8_t size) {

    if (receivedDataSize_ > size) return 0;

    for (uint16_t i = 0; i < receivedDataSize_; i++) buffer[i] = receivedData_[i];

    uint8_t sizePacket = receivedDataSize_;
    receivedDataSize_ = 0;
    
    return sizePacket;

}