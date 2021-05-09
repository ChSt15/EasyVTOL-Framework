#include "kraft_konnect_network.h"



void KraftKonnectNetwork::thread() {

    if (block_) return;

    loopCounter_++;


    if (commsPort_ == nullptr) return;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        commsPort_->loop();

        if (!commsPort_->networkBusy() && heartbeatInterval_.isTimeToRun()) {

            KraftMessageHeartbeat message;
            commsPort_->sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        }

        if (commsPort_->messageAvailable()) {

            MessageData messageData = commsPort_->getMessageInformation();

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

    moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

}
