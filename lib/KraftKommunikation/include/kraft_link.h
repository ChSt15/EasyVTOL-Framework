#ifndef KRAFT_LINK_H
#define KRAFT_LINK_H



#include "stdint.h"



/**
 * Abstract class used for interfacing with radio modules
 */
class KraftLink_Interface {
public:
    

    /**
     * Checks if radio is busy or can send a new data packet.
     * 
     * @returns true if radio is busy.
     */
    virtual bool busy() = 0;


    /**
     * Gives radio data to send.
     * 
     * @param buffer is a pointer to a uint8_t* array containing all data to be sent.
     * @param size is a uint8_t integer giving the amount of data to send from the buffer pointer.
     * @returns number of bytes sent. Will be 0 if failed.
     */
    virtual uint8_t sendBuffer(uint8_t* buffer, uint8_t size) = 0;


    /**
     * Checks if data is available for reading
     * 
     * @returns number of bytes received ready for reading. Will be 0 if none available
     */
    virtual uint8_t available() = 0;

    /**
     * Places received data into buffer and returns number of bytes placed into buffer
     * 
     * @param buffer is a pointer to a uint8_t* array where all the data is to be placed.
     * @param size is the max size of buffer.
     * @returns number of bytes placed into buffer.
     */
    virtual uint8_t receiveBuffer(uint8_t* buffer, uint8_t size) = 0;


};



#endif 
