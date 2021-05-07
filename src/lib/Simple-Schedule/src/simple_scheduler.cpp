#include "simple_scheduler.h"



/**
 * This is the loop for the scheduler.
 * It will check for functions that need to be ran and run them.
 */
void Scheduler::tick() {

    yield();

    //Measure and calculate tickrate
    tickCounter_++;  
    uint32_t dTime;
    if (tickCounterResetInterval_.isTimeToRun(dTime)) {
        tickRate_ = (double)tickCounter_/dTime*1000000.0;
        tickCounter_ = 0;
    }

    //Run all tasks with a priority. If none are found then go to next lower priority.
    ChainObject<Task>* currentTask = tasks_[0].getChainStart();
    bool taskRan = false; //Set to true if a task was run. Will only be checked if we switch to lower task.
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    //Start with functions without parameters
    currentTask = tasks_[1].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    currentTask = tasks_[2].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    currentTask = tasks_[3].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    currentTask = tasks_[4].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    currentTask = tasks_[5].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

    if (taskRan) return; //Check if a task ran. If so then leave.
    currentTask = tasks_[6].getChainStart(); //Switch to next priority
    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
                taskRan = true;
            }
            if (currentTask->item.limited) {
                if (micros() - currentTask->item.creationTimestamp_us >= currentTask->item.removeThreshold_us) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) currentTask->item.numberRunsLeft--;
                if (currentTask->item.numberRunsLeft == 0) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            }
        //}
        currentTask = nextTask;

    }
    //currentRunningTask_ = nullptr; //Set to nullpointer when leaving as nothing is running

}

void Scheduler::initializeTasks() {

    for (uint8_t i = 0; i < 7; i++) {

        ChainObject<Task>* currentTask = tasks_[i].getChainStart(); //Switch to next priority

        while(currentTask != nullptr) {
            ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed

            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
            }
            
            currentTask = nextTask;

        }

    }


}

/**
 * This adds a function to the scheduler.
 * 
 * Functions cannot return.
 * 
 * Will return false if fails to add function to scheduler.
 * 
 * e.g. attachTask(FunctionToBeCalled, 1000, eTaskPriority_t::eTaskPriority_Realtime);
 *
 * @param values function pointer, rate_Hz, priority
 * @return bool.
 */
bool Scheduler::attachTask(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority, int32_t numberRuns) {

    counter++;

    Task task;
    task.thread = function;
    task.interval = IntervalControl(rate_Hz);
    task.interval.setLimit(false);
    task.numberRunsLeft = numberRuns;

    switch (priority) {

    case eTaskPriority_t::eTaskPriority_Realtime:
        tasks_[0].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_VeryHigh :
        tasks_[1].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_High :
        tasks_[2].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_Middle :
        tasks_[3].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_Low :
        tasks_[4].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_VeryLow :
        tasks_[5].addItem(task);
        break;
    
    default:
        tasks_[6].addItem(task);
        break;
    }

    return true;
}


/**
 * The given function will be ran at the given time.
 * 
 * 
 * Functions cannot return.
 * 
 * Will return false if fails to add function to scheduler.
 * 
 * e.g. attachTask(FunctionToBeCalled, 1000, eTaskPriority_t::eTaskPriority_Realtime);
 *
 * @param values function pointer, rate_Hz, priority
 * @return bool.
 */
bool Scheduler::attachTask(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority, uint32_t time_us, int32_t numberRuns) {

    Task task;
    task.thread = function;
    task.interval = IntervalControl(rate_Hz);
    //task.interval.setLimit(false);
    task.creationTimestamp_us = micros();
    task.removeThreshold_us = time_us;
    task.numberRunsLeft = numberRuns;
    task.limited = true;

    switch (priority) {

    case eTaskPriority_t::eTaskPriority_Realtime:
        tasks_[0].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_VeryHigh :
        tasks_[1].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_High :
        tasks_[2].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_Middle :
        tasks_[3].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_Low :
        tasks_[4].addItem(task);
        break;

    case eTaskPriority_t::eTaskPriority_VeryLow :
        tasks_[5].addItem(task);
        break;
    
    default:
        tasks_[6].addItem(task);
        break;
    }

    return true;
}

/**
 * This removes a function from the scheduler.
 * 
 * Returns false if function not found.
 *
 * @param values none.
 * @return none.
 */
bool Scheduler::detachTask(Thread_Interface* function) {

    Task toBeRemoved;
    toBeRemoved.thread = function;

    for (uint8_t i = 0; i < 7; i++) {
        if (tasks_[i].removeItem(toBeRemoved)) {
            return true;
        }
    }

    return false;
}
