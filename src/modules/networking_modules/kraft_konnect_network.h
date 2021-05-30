#ifndef KRAFTKONNECT_NETWORK_DRIVER_H
#define KRAFTKONNECT_NETWORK_DRIVER_H



#include "Arduino.h"

#include "lib/KraftKommunikation/src/kraft_kommunication.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "modules/module_abstract.h"

#include "KraftPacket_KontrolPackets/kraftkontrol_message_types.h"







class KraftKonnectNetwork: public Module_Abstract, public Task_Abstract {
public:

    KraftKonnectNetwork(KraftKommunication* communicationPort) : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_Middle, true) {
        commsPort_ = communicationPort;
    }
    
    /**
     * This is where all calculations are done.
     */
    void thread();

    /**
     * Init function that sets the module up.
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @returns rate that thread is ran at.
     */
    uint32_t loopRate() {return loopRate_;};

    /**
     * Given function will be called when message is received.
     * 
     * @param eventHandler Pointer to the function to be called.
     * @param eventMessageType What message to trigger event. Should be a message type
     */
    void setEventHandler(void (*eventHandler)(void), uint8_t eventMessageType) {eventHandlers_[constrain(eventMessageType, 0, 255)] = eventHandler;}

    /**
     * Given function will be called when message is received.
     * 
     * @param eventHandler Pointer to the function to be called.
     * @param eventMessageType What message to trigger event. Should be a message type
     */
    void setNodeTimeoutHandler(void (*eventHandler)(void), uint8_t nodeID) {timeoutEventHandlers_[constrain(nodeID, 0, 255)] = eventHandler;}


private:

    //Receive handlers. These are called when a certain message type was received.
    void (*eventHandlers_[255])(void);
    //Timeout handlers. These are called when communication with a node was lost. (no more heartbeats received)
    void (*timeoutEventHandlers_[255])(void);

    //Timestamps for last received messages from a node
    uint32_t nodeTimestamp_[255];
    bool nodeConnected_[255];


    KraftKommunication* commsPort_ = nullptr;

    IntervalControl rateCalcInterval_ = IntervalControl(1);   
    IntervalControl heartbeatInterval_ = IntervalControl(5);

    uint8_t startAttempts_ = 0;

    uint32_t loopRate_ = 0;
    uint32_t loopCounter_ = 0;

    uint32_t lastMeasurement_ = 0;

    bool block_ = false;

    
};





#endif