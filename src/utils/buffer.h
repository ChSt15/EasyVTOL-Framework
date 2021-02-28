#ifndef BUFFER_HELPER_H
#define BUFFER_HELPER_H


/**
 * This class implements a queue and stack type buffer.
 * This can also sort stored values and be used for processing measurements
*/



#include "stdint.h"
#include "math.h"



template<typename T, uint32_t S> // <sample type, sample size>
class Buffer {
public:

    /**
     * Constructor for Buffer class.
     * <type, size> 
     * "Type" is the data type (uint32_t, int float etc.)
     * "size" is the maximum number of samples that will be stored.
     *
     * @param values none.
     * @return none.
     */
    Buffer();

    ~Buffer(); //Remember to delete data on destruction

    /**
     * Places a new element to the front of the buffer.
     * 
     * Returns true when successfull without overwriting data.
     * False when overwriting data at back.
     *
     * @param values element.
     * @return bool.
     */
    bool placeFront(const T &element) {
        
    }

    /**
     * Places a new element to the back of the buffer.
     * 
     * Returns true when successfull without overwriting data.
     * False when overwriting data at front.
     *
     * @param values element.
     * @return bool.
     */
    bool placeBack(const T &element) {
        
    }

    /**
     * Takes a element from the front of the buffer.
     * Element will be removed.
     * peekFront() wont remove element.
     *
     * @param values none.
     * @return element.
     */
    T takeFront() {
        
    }

    /**
     * Take a element from the back of the buffer.
     * Element will be removed.
     * peekBack() wont remove element.
     *
     * @param values none.
     * @return element.
     */
    T takeBack() {
        
    }

    /**
     * Returns a copy of the element from the front of the buffer.
     * Element will not be removed.
     *
     * @param values none.
     * @return element.
     */
    T peekFront() {
        
    }

    /**
     * Returns a copy of the element from the front of the buffer.
     * Element will not be removed.
     *
     * @param values none.
     * @return element.
     */
    T peekBack() {
        
    }

    /**
     * Returns the sum of all elements.
     *
     * @param values none.
     * @return sum of elements.
     */
    T getSum() {
        
    }

    /**
     * Returns the average of all elements.
     *
     * @param values none.
     * @return sum of elements.
     */
    T getAverage() {
        
    }

    /**
     * Sorts the elements. 
     * This takes a lot of processing power!
     *
     * @param values none.
     * @return none.
     */
    void sortElements() {
        
    }

    /**
     * Sorts elements and returns median.
     * This takes a lot of processing power!
     *
     * @param values none.
     * @return median.
     */
    T getMedian() {
        
    }

    /**
     * Calculates the standard deviation.
     *
     * @param values none.
     * @return deviation.
     */
    T getStandardDeviation() {
        
    }

    /**
     * Calculates the standard error.
     *
     * @param values none.
     * @return deviation.
     */
    T getStandardError() {
        
    }

    /**
     * Used to access buffer like an array.
     * Indexes outside of buffer will wrap around
     * buffer.
     */
    Buffer operator[] (const uint32_t &index) {

    }

    /**
     * Needs to be overloaded to also copy the data to instance.
     * Not doing this will cause 2 buffers to share the same elements.
     */
    Buffer operator = (const Buffer &index) {

    }


private:

    void _sortElements(T* elements, uint32_t size);


    uint32_t numElements = S;

    T* elementPointer = nullptr;

    uint32_t front = 0;
    uint32_t back = 0;


};











#endif