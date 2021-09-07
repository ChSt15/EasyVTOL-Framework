#ifndef SX1280_LORA_DRIVER_H
#define SX1280_LORA_DRIVER_H



#include "Arduino.h"
#include "SPI.h"

#include "KraftKontrol/utils/KraftKommunikation/kraft_kommunication.h"
#include "lib/SX12XX-LoRa-master/src/SX128XLT.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/modules/module_abstract.h"



//Lora radio settings.
#define SX1280_FREQUENCY        2445000000UL    
#define SX1280_SPREADFACTOR     LORA_SF5        
#define SX1280_BANDWIDTH        LORA_BW_1600
#define SX1280_CODINGRATE       LORA_CR_4_5

//Keep at 10mW to stay legal. 100mW is allowed in europe only if spread spectrum is implemented. (It currently isnt so dont)
#define SX1280_POWER_dB         10       

//This id the size of the buffer used to store data that was recieved and to be sent.
#define SX1280_DATA_BUFFER_SIZE 255

//Uncomment this to enable serial data output
//#define SX1280_DEBUG  





class SX1280Driver: public KraftLink_Interface, public Module_Abstract, public Task_Abstract {
public:

    SX1280Driver(int busyPin, int txenPin, int rxenPin, int dio1Pin, int nResetPin, int nssPin/*, SPIClass* spiBus*/) : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_Middle, true) {
        nssPin_ = nssPin;
        busyPin_ = busyPin;
        txenPin_ = txenPin;
        rxenPin_ = rxenPin;
        dio1Pin_ = dio1Pin;
        resetPin_ = nResetPin;
        //spiBus_ = spiBus;
    }
    
    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t loopRate() {return loopRate_;};

    /**
     * @returns true if the internal buffer is full and adding a new message will result in data loss.
     */
    bool busy() const override {return toSendBufferSub_.availableSpace() == 0;}

    /**
     * Get the last received data SNR. Used for signal strength. Better than RSSI
     * +10 is strong -20 is at limit.
     * 
     * @returns last received datas SNR
     */
    int8_t getSNR() {return receivedDataSNR_;}

    /**
     * Get the last received data RSSI. Used for signal strength.
     * 
     * @returns last received datas RSSI
     */
    int8_t getRSSI() {return receivedDataRSSI_;}


private:

    void internalLoop();

    //Buffer for data that was received by radio
    uint8_t receivedData_[SX1280_DATA_BUFFER_SIZE];
    uint8_t receivedDataSize_ = 0;
    int8_t receivedDataSNR_ = -100;
    int8_t receivedDataRSSI_ = -100;
    //Buffer for data that is to be sent by radio
    uint8_t toSendData_[SX1280_DATA_BUFFER_SIZE];
    uint8_t toSendDataSize_ = 0;

    //Should be true whileits waiting for send to complete
    bool isBusySending_ = false;

    IntervalControl rateCalcInterval_ = IntervalControl(1); 

    Buffer_Subscriber<DataMessageBuffer, 10> toSendBufferSub_ = Buffer_Subscriber<DataMessageBuffer, 10>(toSendDataTopic_);

    int nssPin_;
    int busyPin_;
    int resetPin_;
    int txenPin_;
    int rxenPin_;
    int dio1Pin_;
    //SPIClass* spiBus_;
    SX128XLT radio_;      

    uint8_t startAttempts_ = 0;

    uint32_t loopRate_ = 0;
    uint32_t loopCounter_ = 0;

    uint32_t lastMeasurement_ = 0;

    bool block_ = false;

    
};





#endif