#ifndef TOPIC_SUBSCRIBERS_H
#define TOPIC_SUBSCRIBERS_H


#include "topic.h"
#include "buffer.h"

#include "Simple-Schedule/task_autorun_class.h"



/**
 * Subscribes to topic(s). USed by other subscribers to implement core functions.
 */
template<typename TYPE> 
class Subscriber_Generic: public Subscriber_Interface<TYPE> {
friend Topic<TYPE>;
public: 

    Subscriber_Generic() {}

    Subscriber_Generic(Topic<TYPE>& topic) {
        subscribe(topic);
    }

    virtual ~Subscriber_Generic() {}

    /**
     * Will remove all subscribtions.
     */
    void unsubcribe() override {
        for (uint32_t i = 0; i < subscribedTopics_.getNumItems(); i++) {
            subscribedTopics_[i]->removeSubscriber(this);
        }
        subscribedTopics_.clear();
    }

    /**
     * Will remove subscribtion. Will not receive anymore.
     */
    void unsubcribeTopic(const Topic<TYPE>& topic) {
        topic.removeSubscriber(this);
        subscribedTopics_.removeAllEqual(&topic);
    }

    /**
     * Subscribes to given topic. Will remove old subscription.
     * @param topic Topic to subscribe to.
     */
    void subscribe(const Topic<TYPE>& topic) override {
        topic.addSubscriber(this);
        subscribedTopics_.removeAllEqual(&topic);
        subscribedTopics_.append(&topic);
    }

    /**
     * Publishes an item to topic, but will not receive item. Makes it simpler to broadcast items from modules.
     * @param item Item to publish
     */
    /*void publish(TYPE& item) {
        for (uint32_t i = 0; i < subscribedTopics_.getNumItems(); i++) subscribedTopics_[i]->publish(item, this);
    }*/


private:

    //List of pointers to all subscribed topics.
    List<const Topic<TYPE>*> subscribedTopics_;

};


/**
 * This header contains the implementation of a few subscriber classes.
 * 
 * Simple_Subscriber is fastest and only contains one item.
 * Buffer_Subscriber is identical to FiFoBuffer but auto adds items to beginning of buffer.
 */


/**
 * This subscriber contains only one item that is updated on every publish.
 * @see Buffer_Subscriber for receiving more them one item.
 * Use isDataNew() to check if an item was updated.
 */
template<typename TYPE> 
class Simple_Subscriber: public Subscriber_Generic<TYPE> {
private:

    bool itemIsNew = false;
    bool itemIsValid = false;

    TYPE receivedItem;

    Task_Abstract* taskToResume_ = nullptr;

    void receive(TYPE& item, Topic<TYPE>* topic) override {
        receivedItem = item;
        itemIsNew = true;
        if (taskToResume_ != nullptr) taskToResume_->startTaskThreading();
    }


public:

    Simple_Subscriber() {}

    /**
     * @param topic Topic to subscribe to.
     */
    Simple_Subscriber(Topic<TYPE>& topic): Subscriber_Generic<TYPE>(topic) {}

    /**
     * @returns True if new data was received
     */
    inline bool isDataNew() const {return itemIsNew;}

    /**
     * @returns true if internal data storage has been updated.
     */
    inline bool isValid() const {return itemIsValid;}
    
    /**
     * @returns reference to item.
     */
    inline const TYPE& getItem() {
        itemIsNew = false; 
        return receivedItem;
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

};



/**
 * This subscriber implements a Fifo. New items are placed into Fifo front.
 * @see Simple_Subscriber for receiving only one item and higher perfomance
 * 
 */
template<typename TYPE, uint32_t SIZE> 
class Buffer_Subscriber: public Subscriber_Generic<TYPE>, public Buffer<TYPE, SIZE> {
public:

    Buffer_Subscriber(bool overwrite = false) {}

    /**
     * @param topic Topic to subscribe to.
     * @param overwrite Set to true to overwrite oldest values if full. Defaults to false.
     */
    Buffer_Subscriber(Topic<TYPE>& topic, bool overwrite = false): Subscriber_Generic<TYPE>(topic) {
        overwrite_ = overwrite;
    }

    /**
     * Sets if subscriber should overwrite when fifo is full.
     * @param overwrite Whether to overwrite.
     */
    void setOverwrite(bool overwrite) {overwrite_ = overwrite;}

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

    void receive(TYPE& item, Topic<TYPE>* topic) override {
        this->placeFront(item, overwrite_);
        if (taskToResume_ != nullptr) taskToResume_->startTaskThreading();
    }

    bool overwrite_ = false;

    Task_Abstract* taskToResume_ = nullptr;

};



/**
 * This subscriber calls the given function passing the data received form topic to it.
 */
template<typename TYPE> 
class Callback_Subscriber: public Subscriber_Generic<TYPE> {
public:

    Callback_Subscriber() {}

    /**
     * @param topic Topic to subscribe to.
     * @param callbackFunc Function to call on data receive.
     */
    Callback_Subscriber(Topic<TYPE>& topic, void (*callbackFunc)(TYPE& item)): Subscriber_Generic<TYPE>(topic) {
        callbackFunc_ = callbackFunc;
    }

    /**
     * Will resume given task if an item is recieved.
     * Callback will be called first, then task is resumed.
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

    void receive(TYPE& item, Topic<TYPE>* topic) override {
        if (callbackFunc_ != nullptr) callbackFunc_(item);
        if (taskToResume_ != nullptr) taskToResume_->startTaskThreading();
    }

    void (*callbackFunc_)(TYPE& item) = nullptr;

    Task_Abstract* taskToResume_ = nullptr;


};




#endif