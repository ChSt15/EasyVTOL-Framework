#ifndef SCHEDULER_H
#define SCHEDULER_H



#include "utils/chain_buffer.h"
#include "utils/interval_control.h"



/**
 * Gives a task a priority. 
 * Lower priorities only run if no tasks exist at higher priorities.
 * Tasks with equal priorities will run one after another.
 * Tasks with REALTIME will always run. But must be written to run fast as to give
 * lower priority tasks time to run.
 */ 
enum TASK_PRIORITY {
    //Will only run if there is nothing to do.
    PRIORITY_NONE,
    //Will only run if nothing to do at higher priorities.
    PRIORITY_VERYLOW,
    //Will only run if nothing to do at higher priorities.
    PRIORITY_LOW,
    //Will only run if nothing to do at higher priorities.
    PRIORITY_MIDDLE,
    //Will only run if nothing to do at higher priorities.
    PRIORITY_HIGH,
    //Will only run if nothing to do at higher priorities.
    PRIORITY_VERYHIGH,
    //Will always run once it needs to.
    PRIORITY_REALTIME,
};



class Scheduler {
public:

    /**
     * Scheduler.
     *
     * @param values none.
     * @return none.
     */
    void initialise();

    /**
     * This is the loop for the scheduler.
     * It will check for functions that need to be ran and run them.
     *
     * @param values none.
     * @return none.
     */
    void tick();

    /**
     * This adds a function to the scheduler.
     * 
     * Functions cannot return or require parameters.
     * 
     * Will return false if fails to add function to scheduler.
     * 
     * e.g. attachFunction(FunctionToBeCalled, 1000, TASK_PRIORITY::PRIORITY_REALTIME);
     *
     * @param values function pointer, rate_Hz, priority
     * @return bool.
     */
    bool attachFunction(void (*function)(void), uint32_t rate_Hz, TASK_PRIORITY priority = TASK_PRIORITY::PRIORITY_MIDDLE);

    /**
     * Will run a function for a given amount of time (in microseconds)
     * 
     * If continualRun is false then the function will only be ran 
     * once at its removal.
     * 
     * Functions cannot return or require parameters.
     * 
     * Will return false if fails to add function to scheduler.
     * 
     * e.g. attachFunction(FunctionToBeCalled, 1000, TASK_PRIORITY::PRIORITY_REALTIME);
     *
     * @param values function pointer, rate_Hz, priority
     * @return bool.
     */
    bool attachFunction(void (*function)(void), uint32_t rate_Hz, TASK_PRIORITY priority, uint32_t time_us, bool continualRun = true);

    /**
     * This removes a function from the scheduler.
     * 
     * Returns false if function not found.
     *
     * @param values none.
     * @return none.
     */
    bool detachFunction(void (*function)(void));


private:

    //Strcut for comparing and storing function pointers that will be run.
    struct Task {

        void (*function)(void);
        IntervalControl interval;

        //If not -1 then this task will be removed once it reached its removal threshold.
        uint32_t removeThreshold_us = 0;
        uint32_t creationTimestamp_us = 0;

        //If set to true then remove once remove timestamp is reached.
        bool limited = false;
        //If set to false then function will only be ran when timestamp is reached.
        bool continualRun = true; 

        //Operator must be overloaded for the Chainbuffer to find the correct Task. Only the function pointers must be the same.
        bool operator == (Task b) {
            return function == b.function;
        }

    };

    ChainBuffer<Task> _tasks[7]; //array size must be as big, as the number of priorities there are.

    bool _chainModified[7]; //This is a flag for when the chain was modified. We need this because we are using direct access to the chain objects.


};





#endif
