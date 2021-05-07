#ifndef SIMPLE_SCHEDULER_H
#define SIMPLE_SCHEDULER_H



#include "chain_buffer.h"
#include "interval_control.h"



/**
 * This is an interface used by simple schedule. 
 * Everything that inherets from this class can be attached onto a scheduler.
 */
class Thread_Interface {
public: 

    //This is ran once before first run of thread()
    virtual void init() = 0;

    //This is ran at rate set in scheduler.
    virtual void thread() = 0;

    //This is ran only when the Task is removed from scheduler.
    virtual void removal() = 0;

};



/**
 * This is a class that can be used to attach simple functions to a scheduler. 
 */
class Thread: public Thread_Interface {
public:

    /**
     * Used to construct a Thread class.
     * 
     * @param functionToRunOnInit is a pointer to a function that will be run once before thread
     * @param functionToRunOnThread is a pointer to a function that will be run on thread.
     * @param functionToRunOnRemoval is a function that is only ran once on tast removal. Default value is null and will then be ignored.
     */
    Thread(void (*functionToRunOnInit)(void), void (*functionToRunOnThread)(void), void (*functionToRunOnRemoval)(void) = nullptr) {
        functionToRunOnInit_ = functionToRunOnInit;
        functionToRunOnThread_ = functionToRunOnThread;
        functionToRunOnRemoval_ = functionToRunOnRemoval;
    }

    //This is ran once before thread first run
    virtual void init() {
        if (functionToRunOnInit_ != nullptr) functionToRunOnInit_();
    }

    //This is ran at rate set in scheduler.
    virtual void thread() {
        if (functionToRunOnThread_ != nullptr) functionToRunOnThread_();
    }

    //This is ran only when the Task is removed from scheduler.
    virtual void removal() {
        if (functionToRunOnRemoval_ != nullptr) functionToRunOnRemoval_();
    }

private:
    
    //Pointer to init function
    void (*functionToRunOnInit_)(void) = nullptr;

    //Pointer to thread function
    void (*functionToRunOnThread_)(void) = nullptr;

    //Pointer to removal function
    void (*functionToRunOnRemoval_)(void) = nullptr;
    

};



/**
 * Gives a task a priority. 
 * Lower priorities only run if no tasks exist at higher priorities.
 * Tasks with equal priorities will run one after another.
 * Tasks with REALTIME will always run. But must be written to run fast as to give
 * lower priority tasks time to run.
 */ 
enum eTaskPriority_t {
    //Will only run if there is nothing to do.
    eTaskPriority_None,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_VeryLow,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_Low,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_Middle,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_High,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_VeryHigh,
    //Will always run once it needs to.
    eTaskPriority_Realtime,
};



class Scheduler {
public:

    uint32_t counter = 0;

    /**
     * This is the loop for the scheduler.
     * It will check for functions that need to be ran and run them.
     *
     * @param values none.
     * @return none.
     */
    void tick();

    /**
     * Calls all initialisation functions that are attached
     */
    void initializeTasks();

    /**
     * Calling this will pause the thread until the given time was reached.
     * e.g. waitUntil(micros() + 1000) will delay the program for 1000 microseconds.
     * If no thread is currently running the this will immediatly return. 
     * 
     * This might not be accurate if many threads suspend themselves.
     * 
     * REMOVED DUE TO MANY PROBLEMS THIS WOULD CREATE
     * 
     * @param timeInMicrosecond Time in microseconds at which the thread should continue
     */
    /*void suspendUntil(uint32_t timeInMicrosecond) {

        if (currentRunningTask_ != nullptr) currentRunningTask_->isSuspended = true;

        while(timeInMicrosecond > micros()) {
            tick();
        }

        if (currentRunningTask_ != nullptr) currentRunningTask_->isSuspended = false;

    }*/

    /**
     * This adds a function to the scheduler.
     * 
     * Functions cannot return.
     * 
     * Will return false if fails to add function to scheduler.
     * 
     * e.g. attachTask(FunctionToBeCalled, 1000, eTaskPriority_t::eTaskPriority_realtime);
     *
     * @param values function pointer, rate_Hz, priority
     * @return bool.
     */
    bool attachTask(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority = eTaskPriority_t::eTaskPriority_Middle, int32_t numberRuns = -1);

    /**
     * Will run a function for a given amount of time (in microseconds)
     * 
     * If continualRun is false then the function will only be ran 
     * once at its removal.
     * 
     * Functions cannot return.
     * 
     * Will return false if fails to add function to scheduler.
     * 
     * e.g. attachTask(FunctionToBeCalled, 1000, eTaskPriority_t::eTaskPriority_realtime);
     *
     * @param values function pointer, rate_Hz, priority
     * @return bool.
     */
    bool attachTask(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority, uint32_t time_us, int32_t numberRuns = -1);

    /**
     * This removes a function from the scheduler.
     * 
     * Returns false if function not found.
     *
     * @param values none.
     * @return none.
     */
    bool detachTask(Thread_Interface* function);

    /**
     * Used to get how often per second tick() is called. Can be used to see how the systems performance is.
     * 
     * @returns tick rate.
     */
    uint32_t getTickRate() {return tickRate_;}


private:

    //Struct for comparing and storing function pointers that will be run. This is without parameters
    struct Task {

        Thread_Interface* thread;
        IntervalControl interval;

        //If true then do not run task.
        bool isSuspended = false;

        //To store whether init function was ran.
        bool initWasCalled = false;

        //If not -1 then this task will be removed once it reached its removal threshold.
        uint32_t removeThreshold_us = 0;
        uint32_t creationTimestamp_us = 0;

        //If set to true then remove once remove timestamp is reached.
        bool limited = false;
        //If set to 0 then no limit. Should be decremented every run.
        int32_t numberRunsLeft = 0;

        //Operator must be overloaded for the Chainbuffer to find the correct Task. Only the function pointers must be the same.
        bool operator == (Task b) {
            return thread == b.thread;
        }

    };

    ChainBuffer<Task> tasks_[7]; //array size must be as big, as the number of priorities there are.

    //Points to the thread the is currently running.
    Task* currentRunningTask_ = nullptr;

    //Incremented every tick
    uint32_t tickCounter_ = 0;
    //Stores loopRate
    uint32_t tickRate_ = 0;
    //Used to reset loopcounter and save Looprate
    IntervalControl tickCounterResetInterval_ = IntervalControl(1);


};





#endif
