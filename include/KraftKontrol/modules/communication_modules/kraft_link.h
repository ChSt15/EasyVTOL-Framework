#ifndef KRAFT_LINK_H
#define KRAFT_LINK_H



#include "KraftKontrol/utils/topic_subscribers.h"
#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_data_messages.h"
//#include "KraftKontrol/modules/module_abstract.h"



/**
 * Abstract class used for interfacing with radio modules
 */
class KraftLink_Abstract {
public:

    /**
     * @returns true if the internal buffer is full and adding a new message will result in data loss.
     */
    virtual bool busy() const = 0;

    /**
     * Use this topic to get received data packets.
     * @returns topic of received packets.
     */
    const Topic<DataMessageBuffer>& getReceivedDataTopic() const {return receivedDataTopic_;}

    /**
     * Use this topic to send data packets.
     * @returns topic of data packets to send.
     */
    const Topic<DataMessageBuffer>& getToSendDataTopic() const {return toSendDataTopic_;}


protected:

    //Place received data into this buffer.
    Topic<DataMessageBuffer> receivedDataTopic_; 

    //Send data from this buffer.
    Topic<DataMessageBuffer> toSendDataTopic_; 

};



#endif 
