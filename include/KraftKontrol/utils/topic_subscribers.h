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
    Simple_Subscriber(Topic<TYPE>& topic): Subscriber_Interface<TYPE>(&topic) {}

    /**
     * @returns True if new data was received
     */
    inline bool isDataNew() const {return itemIsNew;}
    
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
    Buffer_Subscriber(Topic<TYPE>& topic, bool overwrite = false): Subscriber_Interface<TYPE>(&topic) {
        overwrite_ = overwrite;
    }

    /**
     * Sets if subscriber should overwrite when fifo is full.
     * @param overwrite Whether to overwrite.
     */
    void setOverwrite(bool& overwrite) {overwrite_ = overwrite;}


private:

    void receive(TYPE& item) override {
        this->placeFront(item, overwrite_);
    }

    bool overwrite_ = false;

};




#endif