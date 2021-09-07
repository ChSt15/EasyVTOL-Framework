#ifndef LIST_KRAFT_H
#define LIST_KRAFT_H


#include "stdint.h"


/**
 * This is a class to create a dynamic buffer.
 * Its size can be changed during runtime.
 * @param TYPE type of data to store in List
 */
template<typename TYPE> 
class List {
public:

    /**
     * @param sizeControl if set to true then the list will automatically reduce the internal size to save space. Setting this to false will reduce heap fragmentation. Defaults to false.
     */
    List(bool sizeControl = false) {
        sizeControl_ = sizeControl;
        array_ = new TYPE;
        maxSize_ = 1;
    }

    ~List() {
        delete[] array_;
    }

    /**
     * @returns current max size.
     */
    inline const uint32_t& getNumItems() const;

    /**
     * @returns current internal array size.
     */
    inline const uint32_t& getInternalArrayLength() const;

    /**
     * Adds a copy of the given item to the list.
     * @param item Item to add to list.
     */
    void append(const TYPE& item);

    /**
     * Removes the item the pointer points to.
     * @param item Pointer to exact item to be removed.
     * @returns true if found and removed. False if not.
     */
    bool removeWithPointer(const TYPE* item);

    /**
     * Removes the item at the given index.
     * @param index Index of item to be removed
     * @returns true if found and removed. False if not.
     */
    bool removeAtIndex(const uint32_t& index);

    /**
     * Removes all items in list equal to given item
     * @param index Index of item to be removed
     * @returns true if found and removed. False if not.
     */
    bool removeAllEqual(const TYPE& item);

    /**
     * Removes all items in list.
     */
    void clear();

    /**
     * @param index Index of itm to be returned.
     * @returns reference to item in list
     */
    TYPE& operator [] (uint32_t index);

    /**
     * @param index Index of itm to be returned.
     * @returns reference to item in list
     */
    const TYPE& operator [] (uint32_t index) const;

    /**
     * @param index Index of itm to be returned.
     * @returns reference to item in list
     */
    List& operator = (const List& list);

    /**
     * This will cut down the size of the array if it is less than half full.
     */
    void reduceSize();


private:

    //Current max size.
    uint32_t maxSize_ = 0;

    //Current size
    uint32_t size_ = 0;

    //Pointer to start of array.
    TYPE* array_ = nullptr;

    //Wether to reduce array size automatically.
    bool sizeControl_ = false;

    /**
     * Changes size to given parameter.
     * Copies items to new array.
     * @param size Size to change to.
     */
    void changeSizeTo(const uint32_t& size);

};



template<typename TYPE> 
void List<TYPE>::reduceSize() {

    if (size_ >= maxSize_/2) return;

    uint32_t newSize = size_;

    while (size_ < maxSize_/2) newSize /= 2;

    changeSizeTo(newSize);

}



template<typename TYPE> 
void List<TYPE>::changeSizeTo(const uint32_t& size) {

    //Leave if already same size
    if (size == maxSize_) return;
    
    //Create new array
    TYPE* newPointer = new TYPE[size];

    //Move contents to new array
    uint32_t i;
    for (i = 0; i < size_ && i < size; i++) {
        newPointer[i] = array_[i];
    }

    //Unallocate old array
    delete[] array_;

    //Switch to new array and params
    array_ = newPointer;
    maxSize_ = size;
    size_ = i;

}



template<typename TYPE> 
inline const uint32_t& List<TYPE>::getNumItems() const {
    return size_;
}



template<typename TYPE> 
inline const uint32_t& List<TYPE>::getInternalArrayLength() const {
    return maxSize_;
}



template<typename TYPE> 
void List<TYPE>::append(const TYPE& item) {

    //Double size if array is too small
    if (maxSize_ == 0) changeSizeTo(1);
    else if (size_ >= maxSize_) changeSizeTo(maxSize_*2);

    array_[size_] = item;

    size_++;
    
}



template<typename TYPE> 
bool List<TYPE>::removeAtIndex(const uint32_t& index) {

    if (index >= size_) return false;

    for (uint32_t i = index; i < size_-1; i++) {
        array_[i] = array_[i+1];
    }

    size_--;

    if (sizeControl_ && size_ <= maxSize_/2) changeSizeTo(maxSize_/2);

    return true;
    
}



template<typename TYPE> 
bool List<TYPE>::removeWithPointer(const TYPE* item) {

    //Find index of item
    for (uint32_t i = 0; i < size_; i++)  {

        if (&(array_[i]) == item) {

            removeAtIndex(i);
            return true;

        }

    }   

    //Was not found, return false.
    return false;

}



template<typename TYPE> 
bool List<TYPE>::removeAllEqual(const TYPE& item) {

    bool foundOne = false;

    //Find index of item
    for (uint32_t i = 0; i < size_; i++)  {

        if (array_[i] == item) {

            removeAtIndex(i);
            foundOne = true;

        }

    }   

    return foundOne;

}



template<typename TYPE> 
void List<TYPE>::clear() {

    if (sizeControl_) changeSizeTo(0);
    else size_ = 0;

}



template<typename TYPE> 
TYPE& List<TYPE>::operator [] (const uint32_t index) {
    return array_[index];
}



template<typename TYPE> 
const TYPE& List<TYPE>::operator [] (const uint32_t index) const {
    return array_[index];
}



template<typename TYPE> 
List<TYPE>& List<TYPE>::operator = (const List& list) {
    
    clear();

    //Copy items into list
    for (uint32_t i = 0; i < list.getNumItems(); i++) append(list[i]);

    return *this;

}




#endif