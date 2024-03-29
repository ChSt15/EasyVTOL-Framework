#ifndef TOPIC_H
#define TOPIC_H


#include "stdint.h"

#include "list.h"



template<typename TYPE> 
class Topic;

template<typename TYPE> 
class Subscriber_Generic;


/**
 * Interface class used by subscribers to receive topic data.
 * void receive(const TYPE& item, const Topic<TYPE>* topic) function must be implemented and will be called when receiving a published item.
 */
template<typename TYPE> 
class Subscriber_Interface {
friend Topic<TYPE>;
private:
    
    //Topic should only give items if this is true.
    bool receiveItems_ = true;


public: 

    Subscriber_Interface() {receiveItems_ = true;}

    virtual ~Subscriber_Interface() {
        //unsubcribe();
    }

    /**
     * Sets the subscriber to receive a item from Topic. Removes need to subscribe and unsubscribe which can be costly.
     * @param isEnabled Set to true to receive items. Defaults to true.
     */
    void receiveEnable(bool isEnabled = true) {
        receiveItems_ = isEnabled;
    }

    /**
     * @returns whether or not receiving is enabled.
     */
    bool isReceiveEnabled() {return receiveItems_;}

    /**
     * Will remove all subscribtions. Will not receive anymore data.
     */
    virtual void unsubcribe() = 0;

    /**
     * Will remove only given. Will not receive anymore data from this topic.
     */
    virtual void unsubcribeTopic(const Topic<TYPE>& topic) = 0;

    /**
     * Subscribes to given topic. Will remove old subscription.
     * @param topic Topic to subscribe to.
     */
    virtual void subscribe(const Topic<TYPE>& topic) = 0;

    /**
     * Publishes an item to topic, but will not receive item. Makes it simpler to broadcast items from modules.
     * @param item Item to publish
     */
    virtual void publish(TYPE const& item) = 0;


protected:

    /**
     * This is called when subscriber is supposed to receive an item.
     * @param item Item that subscriber will receive.
     * @param topic Which topic is calling this receive function.
     */
    virtual void receive(const TYPE& item, const Topic<TYPE>* topic) = 0;

};



template<typename TYPE> 
class Topic {
friend Subscriber_Generic<TYPE>;
public:

    Topic(){}

    virtual ~Topic();

    /**
     * @returns list of all subscribers.
     */
    const List<Subscriber_Interface<TYPE>*>& getSubscriberList() const;

    /**
     * Sends item to all subscribers.
     * @param item Item to be sent.
     */
    void publish(const TYPE& item);

    /**
     * Sends copy of given item to all subscribers.
     * @param item Item to be sent.
     */
    void publish(const TYPE&& item);

    /**
     * @returns a copy of the last published item.
     */
    //const TYPE& getLatestItem(); 


private:

    /**
     * Publishes copy of item to subscribers except for given subscriber.
     * @param item Item to be sent.
     * @param subscriber Subscriber to not receive item
     */
    void publish(const TYPE& item, Subscriber_Interface<TYPE>* subscriber) const ;

    //Is constant to allow modules to return const reference and others can subscribe but are unable to publish.
    void addSubscriber(Subscriber_Interface<TYPE>* subscriber) const;

    //Is constant to allow modules to return const reference and others can subscribe but are unable to publish.
    void removeSubscriber(Subscriber_Interface<TYPE>* subscriber) const;

    //List of subscribers.
    mutable List<Subscriber_Interface<TYPE>*> subscribers_;

    //TYPE latestItem;

};



/*template<typename TYPE> 
const TYPE& Topic<TYPE>::getLatestItem() {
    return latestItem;
}*/



template<typename TYPE> 
Topic<TYPE>::~Topic() {
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->unsubcribeTopic(*this);
}



template<typename TYPE> 
const List<Subscriber_Interface<TYPE>*>& Topic<TYPE>::getSubscriberList() const {
    return subscribers_;
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE& item) {
    //latestItem = item;
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) {
        if (subscribers_[i]->receiveItems_) {
            subscribers_[i]->receive(item, this);
        }
    }
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE&& item) {
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) {
        if (subscribers_[i]->receiveItems_) {
            subscribers_[i]->receive(item, this);
        }
    }
    //for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->receive(item, this);
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE& item, Subscriber_Interface<TYPE>* subscriber) const  {
    
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) {
        if (subscribers_[i] != subscriber && subscribers_[i]->receiveItems_) subscribers_[i]->receive(item, this);
    }

}



template<typename TYPE> 
void Topic<TYPE>::addSubscriber(Subscriber_Interface<TYPE>* subscriber) const {
    //Make sure not to have multiple subscribers. So remove.
    subscribers_.removeAllEqual(subscriber);
    subscribers_.append(subscriber);
}



template<typename TYPE> 
void Topic<TYPE>::removeSubscriber(Subscriber_Interface<TYPE>* subscriber) const {
    subscribers_.removeAllEqual(subscriber);
}



#endif