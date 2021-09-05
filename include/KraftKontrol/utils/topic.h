#ifndef TOPIC_H
#define TOPIC_H


#include "stdint.h"

#include "list.h"



template<typename TYPE> 
class Topic;


template<typename TYPE> 
class Subscriber_Interface {
friend Topic<TYPE>;
public: 

    Subscriber_Interface() {}

    Subscriber_Interface(Topic<TYPE>& topic) {
        subscribe(topic);
    }

    ~Subscriber_Interface() {
        unsubcribe();
    }

    /**
     * Will remove subscribtion. Will not receive anymore.
     */
    void unsubcribe() {
        if (topic_ != nullptr) topic_->removeSubscriber(this);
    }

    /**
     * Subscribes to given topic. Will remove old subscription.
     * @param topic Topic to subscribe to.
     */
    void subscribe(Topic<TYPE>& topic) {
        unsubcribe();
        topic_ = &topic;
        topic_->addSubscriber(this);
    }

    /**
     * Publishes an item to topic, but will not receive item. Makes it simpler to broadcast items from modules.
     * @param item Item to publish
     */
    void publish(TYPE& item) {
        topic_->publish(item, this);
    }


protected:

    /**
     * This is called when subscriber is supposed to receive an item.
     * @param item Item that subscriber will receive.
     */
    virtual void receive(TYPE& item) = 0;

private:

    //Pointer to subscribed topic
    Topic<TYPE>* topic_ = nullptr;

};



template<typename TYPE> 
class Topic {
friend Subscriber_Interface<TYPE>;
public:

    Topic(){}

    ~Topic();

    /**
     * @returns list of all subscribers.
     */
    const List<Subscriber_Interface<TYPE>*>& getSubscriberList() const;

    /**
     * Sends item to all subscribers.
     * @param item Item to be sent.
     */
    void publish(TYPE& item);

    /**
     * Sends copy of given item to all subscribers.
     * @param item Item to be sent.
     */
    void publish(TYPE&& item);

    /**
     * @returns a copy of the last published item.
     */
    const TYPE& getLatestItem(); 


private:

    /**
     * Publishes copy of item to subscribers except for given subscriber.
     * @param item Item to be sent.
     * @param subscriber Subscriber to not receive item
     */
    void publish(TYPE& item, Subscriber_Interface<TYPE>* subscriber);

    void addSubscriber(Subscriber_Interface<TYPE>* subscriber);

    void removeSubscriber(Subscriber_Interface<TYPE>* subscriber);

    //List of subscribers
    List<Subscriber_Interface<TYPE>*> subscribers_;

    TYPE latestItem;

};



template<typename TYPE> 
const TYPE& Topic<TYPE>::getLatestItem() {
    return latestItem;
}



template<typename TYPE> 
Topic<TYPE>::~Topic() {
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->unsubcribe();
}



template<typename TYPE> 
const List<Subscriber_Interface<TYPE>*>& Topic<TYPE>::getSubscriberList() const {
    return subscribers_;
}



template<typename TYPE> 
void Topic<TYPE>::publish(TYPE& item) {
    latestItem = item;
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->receive(item);
}



template<typename TYPE> 
void Topic<TYPE>::publish(TYPE&& item) {
    latestItem = item;
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) subscribers_[i]->receive(latestItem);
}



template<typename TYPE> 
void Topic<TYPE>::publish(TYPE& item, Subscriber_Interface<TYPE>* subscriber) {
    
    for (uint32_t i = 0; i < subscribers_.getNumItems(); i++) {
        if (&(subscribers_[i]) != subscriber) subscribers_[i]->receive(item);
    }

}



template<typename TYPE> 
void Topic<TYPE>::addSubscriber(Subscriber_Interface<TYPE>* subscriber) {
    //Make sure not to have multiple subscribers. So remove.
    subscribers_.removeAllEqual(subscriber);
    subscribers_.append(subscriber);
}



template<typename TYPE> 
void Topic<TYPE>::removeSubscriber(Subscriber_Interface<TYPE>* subscriber) {
    subscribers_.removeAllEqual(subscriber);
}



#endif