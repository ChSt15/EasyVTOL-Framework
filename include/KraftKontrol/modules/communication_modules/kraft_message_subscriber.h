#ifndef KRAFT_MESSAGE_SUBSCRIBER_H
#define KRAFT_MESSAGE_SUBSCRIBER_H



#include "stdint.h"

#include "KraftKontrol/utils/topic_subscribers.h"
#include "KraftKontrol/utils/list.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/modules/communication_modules/kraft_message.h"



/**
 * Class used for receiving certain types of messages. Can receive multiple types of messages.
 * Given messages should never be deleted for the lifetime of the subscriber.
 */
class KraftMessage_Subscriber: public Subscriber_Generic<KraftMessage_Interface> {
private:

    bool dataNew_ = false;

    Task_Abstract* taskToResume_ = nullptr;

    List<KraftMessage_Interface*> receiverItems_;


public:

    KraftMessage_Subscriber() {}

    /**
     * @param receiverItem Item to be written with new data on receive. MUST NOT BE DELETED BEFORE THIS SUBSCRIBER!
     */
    KraftMessage_Subscriber(KraftMessage_Interface& receiverItem) {
        addReceiverMessage(receiverItem);
    }

    /**
     * Adds message to list for receiving. SHOULD NEVER BE DELETED FOR LIFETIME OF SUBSCRIBER!
     * @param receiverItem Which message to receive and where to place it.
     */
    void addReceiverMessage(KraftMessage_Interface& receiverItem) {
        receiverItems_.append(&receiverItem);
    }

    /**
     * Removes message from list for receiving.
     * @param receiverItem Which message to remove. Must be exact item.
     */
    void removeReceiverMessage(KraftMessage_Interface& receiverItem) {
        receiverItems_.removeAllEqual(&receiverItem);
    }

    /**
     * Can only be called once. Will always return false until new data is received.
     * @returns true if new data received.
     */
    bool isDataNew() {
        bool returnVal = dataNew_;
        dataNew_ = false;
        return returnVal;
    }

    /**
     * Will resume given task if an item is recieved.
     */
    void setTaskToResume(Task_Abstract& task) {
        taskToResume_ = &task;
    }

    /**
     * Stops resuming the given task that was being resumed.
     */
    void removeTaskResume() {
        taskToResume_ = nullptr;
    }


private:

    void receive(const KraftMessage_Interface& item, const Topic<KraftMessage_Interface>* topic) override {

        for (uint32_t i = 0; i < receiverItems_.getNumItems(); i++) {

            KraftMessage_Interface* receiverItem_ = receiverItems_[i];
        
            if (item.getMessageType() == receiverItem_->getMessageType() && item.getDataType() == receiverItem_->getDataType() && item.getDataSize() == receiverItem_->getDataSize()) {

                uint8_t buffer[receiverItem_->getDataSize()];

                if (item.getRawData(buffer, receiverItem_->getDataSize()) && receiverItem_->setRawData(buffer, receiverItem_->getDataSize())) {
                    dataNew_ = true;
                    if (taskToResume_ != nullptr) taskToResume_->startTaskThreading();
                }
                
            }

        }

    }


};



#endif 
