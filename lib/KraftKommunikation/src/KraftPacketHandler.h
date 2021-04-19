#ifndef KRAFTPACKET_PACKETIZER_H
#define KRAFTPACKET_PACKETIZER_H



#include "stdint.h"

#include "KraftPacketPacketizer.h"



/**
 * Packet transfer algorithm:
 * ...
 * 
 */



/**
 * Class used for handling packets
 */
class KraftPacketHandler_Template {
public:

    void thread();

    void init();



protected:

    /**
     * Tells radio to send the raw data in the buffer with the size of bufferSize.
     * 
     * Returns false if failure.
     *
     * @param values buffer and bufferSize.
     * @return bool.
     */
    virtual bool packetSend(uint8_t* buffer, uint32_t bufferSize) = 0;

    /**
     * Writes data from radio into buffer with the max size of bufferSize.
     * 
     * Returns the number of bytes written into buffer. 0 indicates error. 
     *
     * @param values buffer and bufferSize.
     * @return uint32_t.
     */
    virtual uint32_t packetReceive(uint8_t* buffer, uint32_t bufferSize) = 0;

    /**
     * Returns true if a packet of data has been received from radio
     *
     * @param values none.
     * @return bool.
     */
    virtual bool packetAvailable() = 0;

    /**
     * Returns true if radio is ready to send another packet.
     *
     * @param values none.
     * @return bool.
     */
    virtual bool packetSendReady() = 0;

    /**
     * Gives radio time to proccess packets and interrupts.
     *
     * @param values none.
     * @return bool.
     */
    virtual void packetThread() = 0;
    


};



#endif 
