#ifndef SX1280_LORA_DRIVER_H
#define SX1280_LORA_DRIVER_H



#include "Arduino.h"
#include "SPI.h"

#include "KraftKontrol/modules/communication_modules/kraft_link.h"
#include "lib/SX12XX-LoRa-master/src/SX128XLT.h"

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "KraftKontrol/modules/module_abstract.h"



//Lora radio settings.
#define SX1280_FREQUENCY        2445000000UL    
#define SX1280_SPREADFACTOR     LORA_SF5        
#define SX1280_BANDWIDTH        LORA_BW_1600
#define SX1280_CODINGRATE       LORA_CR_4_6

//Keep at 10mW.
#define SX1280_POWER_dB         10       

//This id the size of the buffer used to store data that was recieved and to be sent.
#define SX1280_DATA_BUFFER_SIZE 255

//Uncomment this to enable serial data output
//#define SX1280_DEBUG  





class SX1280Driver: public KraftLink_Abstract, public Module_Abstract, public Task_Threading {
public:

    SX1280Driver(int busyPin, int txenPin, int rxenPin, int dio1Pin, int nResetPin, int nssPin, float threadRate = 1000) : Task_Threading("SX1280 Driver", eTaskPriority_t::eTaskPriority_Middle, SECONDS/threadRate) {
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

    /**
     * @brief Changes the internal LoRa parameters
     * 
     */
    void setLoRaParams(uint32_t frequency, uint8_t spreadFactor, uint8_t bandwidth, uint8_t codingRate, bool highSensitivity);

    /**
     * @brief Starts receiving packets. Will draw power.
     * 
     */
    void startReceiving();

    /**
     * @brief Stops receiving packets. Energy saving.
     * 
     */
    void stopReceiving();


private:

    void internalLoop();

    //Buffer for data that was received by radio
    int8_t receivedDataSNR_ = -100;
    int8_t receivedDataRSSI_ = -100;

    //Should be true while its waiting for send to complete
    bool isBusySending_ = false;

    //is true if the channel is busy.
    bool channelBusy_ = false;

    //For giving others time to send data.
    int64_t lastSendTimestamp_ = 0;

    Buffer_Subscriber<DataMessageBuffer, 50> toSendBufferSub_;

    int nssPin_;
    int busyPin_;
    int resetPin_;
    int txenPin_;
    int rxenPin_;
    int dio1Pin_;
    //SPIClass* spiBus_;
    SX128XLT radio_;      

    uint8_t startAttempts_ = 0;

    bool isReceivingEnabled_ = false;

    
};





#endif