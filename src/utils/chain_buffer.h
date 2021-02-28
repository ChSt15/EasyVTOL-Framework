#ifndef CHAIN_BUFFER_H
#define CHAIN_BUFFER_H


/**
 * This class implements a buffer that allows objects to be added to a chain.
 * This is similar to an array but one can continuesly add objects to it.
 * All data is stored on head and will therefore also have performance penalties.
 * Please only use if really needed.
*/



#include "stdint.h"



/** 
 * A class to hold item and chain position information
 */
template<typename T> 
struct ChainObject {

    T item;

    ChainObject* nextObject = nullptr;
    ChainObject* lastObject = nullptr;

};



template<typename T> 
class ChainBuffer {
public:

    ChainBuffer() {

    }

    //Need to remove all objects in chain or else memory leak.
    ~ChainBuffer() {
        ChainObject<T>* pointer = _chainStart; //get chain start.
        while (pointer != nullptr) { //Go down chain deleting all objects till nullptr(end).
            ChainObject<T>* buf = pointer->nextObject;
            //delete pointer;
            pointer = buf;
        }
    }

    /**
     * This adds an item to the end of the chain.
     * 
     * Returns a pointer to the ChainObject containing the added item.
     *
     * @param values none.
     * @return bool.
     */
    ChainObject<T>* addItem(const T &item) {

        ChainObject<T>* newObject = new ChainObject<T>;
        newObject->item = item; //Place item in object;

        if (_chainStart != nullptr) {  //Place item at end of chain.
            _chainEnd->nextObject = newObject;
            newObject->lastObject = _chainEnd;
            _chainEnd = newObject;
        } else { //If no item yet in chain then start chain.
            _chainStart = _chainEnd = newObject;
        }

        _numObjects++;
        return newObject;
    }

    /**
     * Searches for the item and removes it.
     * 
     * As a pointer to exact object wasnt given. It will search 
     * for an item that is similar with == operator.
     * 
     * Returns false if item wasnt found.
     *
     * @param values none.
     * @return bool.
     */
    bool removeItem(const T &item) {
        ChainObject<T>* object = _searchForItem(item);
        if (object == nullptr) return false; //Item wasnt found.
        if (object == _chainStart) { //Item is at start of chain.
            _chainStart = object->nextObject;
            if (_chainStart != nullptr) _chainStart->lastObject = nullptr; //Make sure it isnt a nullptr when updating start of chain
        } else if (object == _chainEnd) {
            _chainEnd = object->lastObject;
            if (_chainEnd != nullptr) _chainEnd->nextObject = nullptr; //Make sure it isnt a nullptr when updating end of chain
        } else {
            object->nextObject->lastObject = object->lastObject;
            object->lastObject->nextObject = object->nextObject;
        }
        delete object;
        _numObjects--;
        return true;
    }

    /**
     * Searches for the item and removes it.
     * 
     * Returns false if item wasnt found.
     *
     * @param values none.
     * @return bool.
     */
    bool removeItem(T *item) {
        ChainObject<T>* object = _searchForItem(item);
        if (object == nullptr) return false; //Item wasnt found.
        if (object == _chainStart) { //Item is at start of chain.
            _chainStart = object->nextObject;
            if (_chainStart != nullptr) _chainStart->lastObject = nullptr; //Make sure it isnt a nullptr when updating start of chain
        } else if (object == _chainEnd) {
            _chainEnd = object->lastObject;
            if (_chainEnd != nullptr) _chainEnd->nextObject = nullptr; //Make sure it isnt a nullptr when updating end of chain
        } else {
            object->nextObject->lastObject = object->lastObject;
            object->lastObject->nextObject = object->nextObject;
        }
        delete object;
        _numObjects--;
        return true;
    }

    /**
     * Removes item at the end of chain.
     *
     * @param values none.
     * @return none.
     */
    void removeItemEnd() {
        ChainObject<T>* object = _chainEnd;
        if (object == nullptr) return;
        _chainEnd = object->lastObject;
        if (_chainEnd != nullptr) _chainEnd->nextObject = nullptr; //Make sure it isnt a nullptr when updating end of chain
        delete object;
        _numObjects--;
    }

    /**
     * Removes item at the start of chain
     *
     * @param values none.
     * @return none.
     */
    void removeItemStart() {
        ChainObject<T>* object = _chainStart;
        if (object == nullptr) return;
        _chainStart = object->nextObject;
        if (_chainStart != nullptr) _chainStart->lastObject = nullptr; //Make sure it isnt a nullptr when updating start of chain
        delete object;
        _numObjects--;
    }
        

    /**
     * Returns a reference to the item, not a copy.
     * 
     * Returns null when out of bounds.
     *
     * @param values place.
     * @return T pointer.
     */
    T* operator[] (const uint32_t &place) {

        ChainObject<T>* pointer = _chainStart;

        for (uint32_t i = 0; pointer != nullptr && i < place; i++) {
            pointer = pointer->nextObject;
        }

        return (pointer==nullptr)?nullptr:&pointer->item; //Hehe. Hope you dont hate compact if statements.

    }

    /**
     * Returns the length of the buffer.
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t length() {
        return _numObjects;
    }


private:

    ChainObject<T>* _searchForItem(const T &item) {
        
        ChainObject<T>* pointer = _chainStart;

        while (pointer != nullptr && pointer->object != item) {
            pointer = pointer->nextObject;
        }

        return pointer;

    }

    /*ChainObject<T>* _searchForItem(T *item) {
        
        ChainObject<T>* pointer = _chainStart;

        while (pointer != nullptr && &pointer->object != item) {
            pointer = pointer->nextObject;
        }

        return pointer;

    }*/

    ChainObject<T>* _chainStart = nullptr;
    ChainObject<T>* _chainEnd = nullptr;

    uint32_t _numObjects = 0;


};











#endif