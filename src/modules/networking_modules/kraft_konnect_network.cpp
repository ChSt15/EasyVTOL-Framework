#include "KraftKontrol/modules/networking_modules/kraft_konnect_network.h"



void KraftKonnectNetwork::thread() {

    if (block_) return;

    loopCounter_++;


    if (commsPort_ == nullptr) return;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        commsPort_->loop();

        //Send heartbeat
        if (!commsPort_->networkBusy() && heartbeatInterval_.isTimeToRun()) {

            KraftMessageHeartbeat message;
            commsPort_->sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        }

        //Check if last message from a node was so long ago that there must be no more connection
        for (uint8_t i = 0; i < 255; i++) {

            if (nodeConnected_[i] && NOW() - nodeTimestamp_[i] >= heartbeatInterval_.getInterval()*3) {
                nodeConnected_[i] = false;

                //Call event handler for timeout
                if (timeoutEventHandlers_[i] != nullptr) timeoutEventHandlers_[i]();

            }

        }

        if (commsPort_->messageAvailable()) {

            MessageData messageData = commsPort_->getMessageInformation();

            //Update timestamp
            nodeTimestamp_[messageData.transmitterID] = NOW();
            nodeConnected_[messageData.transmitterID] = true;

            //Run event handler if given
            if (eventHandlers_[messageData.payloadID] != nullptr) eventHandlers_[messageData.payloadID]();

            if (commsPort_->messageAvailable()) { //Check if the message has been removed. If not then remove. This is to keep unknown messages from blokcing the queue.

                MessageData buf = commsPort_->getMessageInformation();

                if (messageData.transmitterID == buf.transmitterID && messageData.messageCounter == buf.messageCounter) commsPort_->removeMessage();

            } 

        }

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


void KraftKonnectNetwork::init() {

    for (uint16_t i = 0; i < sizeof(eventHandlers_)/sizeof(eventHandlers_[0]); i++) eventHandlers_[i] = nullptr;

    for (uint16_t i = 0; i < sizeof(timeoutEventHandlers_)/sizeof(timeoutEventHandlers_[0]); i++) timeoutEventHandlers_[i] = nullptr;

    for (uint16_t i = 0; i < sizeof(nodeTimestamp_)/sizeof(nodeTimestamp_[0]); i++) nodeTimestamp_[i] = 0;

    for (uint16_t i = 0; i < sizeof(nodeConnected_)/sizeof(nodeConnected_[0]); i++) nodeConnected_[i] = false;

    moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

}
