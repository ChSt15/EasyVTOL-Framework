#ifndef TOPIC_SUBSCRIBERS_H
#define TOPIC_SUBSCRIBERS_H


#include "topic.h"
#include "buffer.h"


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
class Simple_Subscriber: public Subscriber_Interface<TYPE> {
public:

    Simple_Subscriber() {}

    /**
     * @param topic Topic to subscribe to.
     */
    Simple_Subscriber(Topic<TYPE>& topic): Subscriber_Interface<TYPE>(topic) {}

    /**
     * @returns True if new data was received
     */
    inline const bool& isDataNew() const {return itemIsNew;}

    /**
     * @returns true if internal data storage has been updated.
     */
    inline const bool& isValid() const {return itemIsValid;}
    
    /**
     * @returns reference to item.
     */
    inline const TYPE& getItem() {
        itemIsNew = false; 
        return receivedItem;
    }


private:

    void receive(TYPE& item) override {
        receivedItem = item;
        itemIsNew = true;
    }

    bool itemIsNew = false;
    bool itemIsValid = false;

    TYPE receivedItem;

};



/**
 * This subscriber implements a Fifo. New items are placed into Fifo front.
 * @see Simple_Subscriber for receiving only one item and higher perfomance
 * 
 */
template<typename TYPE, uint32_t SIZE> 
class Buffer_Subscriber: public Subscriber_Interface<TYPE>, public Buffer<TYPE, SIZE> {
public:

    Buffer_Subscriber(bool overwrite = false) {}

    /**
     * @param topic Topic to subscribe to.
     * @param overwrite Set to true to overwrite oldest values if full. Defaults to false.
     */
    Buffer_Subscriber(Topic<TYPE>& topic, bool overwrite = false): Subscriber_Interface<TYPE>(topic) {
        overwrite_ = overwrite;
    }

    /**
     * Sets if subscriber should overwrite when fifo is full.
     * @param overwrite Whether to overwrite.
     */
    void setOverwrite(bool overwrite) {overwrite_ = overwrite;}


private:

    void receive(TYPE& item) override {
        this->placeFront(item, overwrite_);
    }

    bool overwrite_ = false;

};



/**
 * This subscriber connects 2 topics together by forwarding messages from topic A to B.
 * Use 2 of these to get bidirectional forwarding.
 */
template<typename TYPE> 
class TopicConnection_Subscriber: public Subscriber_Interface<TYPE> {
public:

    TopicConnection_Subscriber() {}

    /**
     * @param topicA Topic to forward data from.
     * @param topicB Topic to forward data to.
     */
    TopicConnection_Subscriber(Topic<TYPE>& topicA, Topic<TYPE>& topicB): Subscriber_Interface<TYPE>(topicA) {
        topicB_ = topicB;
    }

    /**
     * Changes the topic where data in forwarded to.
     * @param topicB Topic to forward data to.
     */
    void forwardTo(Topic<TYPE>& topicB) {
        topicB_ = topicB;
    }


private:

    void receive(TYPE& item) override {
        topicB_.publish(item);
    }

    Topic<TYPE>& topicB_;


};



/**
 * This subscriber calls the given function passing the data received form topic to it.
 */
template<typename TYPE> 
class Callback_Subscriber: public Subscriber_Interface<TYPE> {
public:

    Callback_Subscriber() {}

    /**
     * @param topic Topic to subscribe to.
     * @param callbackFunc Function to call on data receive.
     */
    Callback_Subscriber(Topic<TYPE>& topic, void (*callbackFunc)(TYPE& item)): Subscriber_Interface<TYPE>(topic) {
        callbackFunc_ = callbackFunc;
    }


private:

    void receive(TYPE& item) override {
        if (callbackFunc_ != nullptr) callbackFunc_(item);
    }

    void (*callbackFunc_)(TYPE& item) = nullptr;


};




#endif