#include "KraftKontrol/modules/datalink_modules/sx1280_lora_driver.h"



void SX1280Driver::thread() {


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        internalLoop();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {

        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        
        removeFromScheduler();

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

            if (packetL > 0) { // make sure packet is okay

                receivedDataRSSI_ = radio_.readPacketRSSI();
                receivedDataSNR_ = radio_.readPacketSNR();

                

                radio_.startReadSXBuffer(0);

                for (uint32_t i = 0; i < packetL;) {

                    uint32_t size = radio_.readUint8();

                    DataMessageBuffer receivedData;

                    for (uint32_t j = 0; j < size && j < SX1280_DATA_BUFFER_SIZE; j++) receivedData.getBuffer()[j] = radio_.readUint8();
                    receivedData.setBufferSize(size);

                    //Place inside buffer and publish.
                    receivedDataTopic_.publish(receivedData);

                    i += size;

                }

                radio_.endReadSXBuffer();


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
            lastSendTimestamp_ = NOW();

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says tx done!");
            #endif

        }

        if (irqStatus & (IRQ_TX_TIMEOUT)) {

            isBusySending_ = false;
            lastSendTimestamp_ = NOW();

            #ifdef SX1280_DEBUG
                Serial.println("Interrupt says tx timeout!");
            #endif

        }

        if (irqStatus & (IRQ_CAD_ACTIVITY_DETECTED)) {

            channelBusy_ = true;

            #ifdef SX1280_DEBUG
                Serial.println("Channel is now busy!");
            #endif

        }

        if (irqStatus & (IRQ_CAD_DONE)) {

            channelBusy_ = false;
            lastSendTimestamp_ = NOW();

            #ifdef SX1280_DEBUG
                Serial.println("Channel is now busy!");
            #endif

        }

        radio_.clearIrqStatus(IRQ_RADIO_ALL); 

        //radio_.receive(receivedData_, SX1280_DATA_BUFFER_SIZE, 0, NO_WAIT);
        radio_.receiveSXBuffer(0, 0, NO_WAIT);

    }


    
    if (toSendBufferSub_.available() > 0 && !isBusySending_ && !channelBusy_ && NOW() - lastSendTimestamp_ > 5*MILLISECONDS) { 

        isBusySending_ = true;

        uint8_t buffer[255];
        uint32_t bufferSize = 0;

        uint32_t i;
        for (i = 0; bufferSize + toSendBufferSub_[i].getBufferSize() + 1 < 220 && i < toSendBufferSub_.available(); i++) {
            
            buffer[bufferSize] = toSendBufferSub_[i].getBufferSize();
            memcpy(buffer + bufferSize + 1, toSendBufferSub_[i].getBuffer(), toSendBufferSub_[i].getBufferSize());
            bufferSize += toSendBufferSub_[i].getBufferSize() + 1;

        }

        radio_.transmit(buffer, bufferSize, 0, SX1280_POWER_dB, NO_WAIT);

        for (uint32_t j = 0; j < i; j++) toSendBufferSub_.removeFront();

        #ifdef SX1280_DEBUG
            Serial.println(String("Sent ") + i + " packets at once! Size: " + bufferSize + " bytes.");
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

        radio_.receiveSXBuffer(0, 0, NO_WAIT);

        Serial.println("SX1280 start success!");

        toSendBufferSub_.subscribe(toSendDataTopic_);

        startAttempts_ = 0;

    } else {

        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 

        Serial.println("SX1280 start failure!");

    }

    startAttempts_++;

    if (startAttempts_ >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}