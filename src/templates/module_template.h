#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



#include "Arduino.h"

/**
 * This is a template that contains all the functions that all 
 * modules must have.
*/



class Module {
public:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void thread() = 0;

    /**
     * Module specific init function that sets the 
     * module up.
     *
     * @param values none.
     * @return none.
     */
    virtual void init() = 0;

    /**
     * Will run all module threads that need to run.
     * 
     * CURRENTLY NOT WORKING. Will simply return.
     *
     * @param values none.
     * @return none.
     */
    static void moduleThreadControlLoop() {

        return;

        //std::cout << "test1" << std::endl;

        Module* thread = nullptr;

        while(thread != _firstModule) {

            thread = _nextModuleToRun;

            if (thread == nullptr) thread = _nextModuleToRun = _firstModule; //Make sure we have a valid module to run
            if (thread == nullptr) return; //No modules exist to run or have been fractured off the module chain. This would be bad but not breaking, just no modules auto running.

            if (thread->_runCounter >= thread->_priority) {
                thread->thread(); //Run thread of module when counter reaches priority.
                thread->_runCounter = 0; //Reset counter
            } else {
                thread->_runCounter++; //Increment counter
            }

            thread = _nextModuleToRun = _nextModuleToRun->_getNextModule(); //Get the next module.
            if (_nextModuleToRun == nullptr) thread = _nextModuleToRun = _firstModule; //If no next module then start at beginning of chain

        }

    }


    static uint16_t getNumThreads() {return _threadCount;}


    


protected:

    ~Module() {
        _removeThread();
    }

    /**
     * Calling this will add this module to the thread
     * control and will be run according to its priority.
     * Priority says how often a thread will be run.
     * The lower the more often it will be run.
     * 
     * WARNING!
     * Make sure to call _removeThread when removing a module. 
     * This should be done automatically, but it wont hurt to
     * call it again.
     * 
     * Returns false if failure to attach thread to chain or thread is already attached.
     */
    bool _connectThread(int16_t priority = 0) {

        if (_threadCount == 0) {

            _firstModule = this;
            _nextModuleToRun = this;
            _threadCount++;

        } else {

            Module* pointer = _firstModule;
            if (this == pointer) return false; //Make sure thread isnt already in chain.

            Module* nextPointer = pointer->_getNextModule();

            while(nextPointer != nullptr) { //Go down chain till end. This it to find last module
                pointer = nextPointer;
                nextPointer = pointer->_getNextModule();
            }

            if (pointer == nullptr) return false; //Sanity check. Ya never know.

            this->_setPrevModule(pointer);

            if (pointer != nullptr) pointer->_setNextModule(this);

            _threadCount++;

        }

        return true;

    }

    /**
     * Calling this will remove this module from the thread
     * control.
     * 
     * WARNING!
     * Make sure to call _removeThread when removing a module. 
     * This should be done automatically, but it wont hurt to
     * call it again. 
     * This can happen if the destructor is overloaded.
     */
    void _removeThread() {

        Module* next = this->_getNextModule();
        Module* prev = this->_getPrevModule();

        if (this == _firstModule) {
            _firstModule = next;
        }

        if (this == _nextModuleToRun) _nextModuleToRun = this->_getNextModule(); //Make sure the current thread running pointer is set to a valid module.
        if (_nextModuleToRun == nullptr) _nextModuleToRun = _firstModule; //If nullptr then set to first module. Could be from end of chain or some wierd failure.

        if (next != nullptr) next->_setPrevModule(prev);
        if (prev != nullptr) prev->_setNextModule(next);

        if (_threadCount > 0) _threadCount--;

    }

private:


    Module* _getNextModule() {return _nextModule;}
    Module* _getPrevModule() {return _prevModule;}
    void _setNextModule(Module* modulePointer) {_nextModule = modulePointer;}
    void _setPrevModule(Module* modulePointer) {_prevModule = modulePointer;}

    static uint16_t _threadCount;
    static Module* _nextModuleToRun;
    static Module* _firstModule;

    uint8_t _runCounter = 0;
    uint8_t _priority = 0;

    Module* _nextModule = nullptr;
    Module* _prevModule = nullptr;


};




#endif