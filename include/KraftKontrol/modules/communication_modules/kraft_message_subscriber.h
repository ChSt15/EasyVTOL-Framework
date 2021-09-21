#ifndef KRAFT_MESSAGE_SUBSCRIBER_H
#define KRAFT_MESSAGE_SUBSCRIBER_H



#include "KraftKontrol/utils/topic_subscribers.h"

#include "KraftKontrol/modules/communication_modules/kraft_message.h"



/**
 * Class used for receiving certain types of messages.
 * Given message should never be deleted for the lifetime of the subscriber.
 */
class KraftMessage_Subscriber: public Subscriber_Generic <KraftMessage_Interface> {
private:

    KraftMessage_Interface& receiverItem_;

    bool dataNew_ = false;


public:

    /**
     * @param receiverItem Item to be written with new data on receive. MUST NOT BE DELETED BEFORE THIS SUBSCRIBER!
     */
    KraftMessage_Subscriber (KraftMessage_Interface& receiverItem): receiverItem_(receiverItem) {}


    /**
     * Can only be called once. Will always return false until new data is received.
     * @returns true if new data received.
     */
    bool isDataNew() {
        bool returnVal = dataNew_;
        dataNew_ = false;
        return returnVal;
    }


private:

    void receive(const KraftMessage_Interface& item, const Topic<KraftMessage_Interface>* topic) override {
        
        if (item.getMessageType() == receiverItem_.getMessageType() && item.getDataType() == receiverItem_.getDataType() && item.getDataSize() == receiverItem_.getDataSize()) {
            
            uint8_t buffer[receiverItem_.getDataSize()];

            if (item.getRawData(buffer, receiverItem_.getDataSize()) && receiverItem_.setRawData(buffer, receiverItem_.getDataSize())) dataNew_ = true;
            
        }

    }


};



#endif 
