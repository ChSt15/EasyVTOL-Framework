#ifndef TOPIC_H
#define TOPIC_H


#include "stdint.h"

#include "list.h"



template<typename TYPE> 
class Topic;

template<typename TYPE> 
class Subscriber_Generic;


template<typename TYPE> 
class Subscriber_Interface {
friend Topic<TYPE>;
public: 

    virtual ~Subscriber_Interface() {
        //unsubcribe();
    }

    /**
     * Will remove subscribtion(s). Will not receive anymore data.
     */
    virtual void unsubcribe() = 0;

    /**
     * Subscribes to given topic. Will remove old subscription.
     * @param topic Topic to subscribe to.
     */
    virtual void subscribe(const Topic<TYPE>& topic) = 0;

    /**
     * Publishes an item to topic, but will not receive item. Makes it simpler to broadcast items from modules.
     * @param item Item to publish
     */
    virtual void publish(const TYPE& item) = 0;


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
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->unsubcribe();
}



template<typename TYPE> 
const List<Subscriber_Interface<TYPE>*>& Topic<TYPE>::getSubscriberList() const {
    return subscribers_;
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE& item) {
    //latestItem = item;
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->receive(item, this);
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE&& item) {
    //TYPE itemCopy = item;
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->receive(item, this);
}



template<typename TYPE> 
void Topic<TYPE>::publish(const TYPE& item, Subscriber_Interface<TYPE>* subscriber) const  {
    
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) {
        if (subscribers_[i] != subscriber) subscribers_[i]->receive(item, this);
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