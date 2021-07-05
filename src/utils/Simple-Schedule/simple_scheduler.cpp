#include "KraftKontrol/utils/Simple-Schedule/simple_scheduler.h"



void Scheduler::tick() {

    //yield();

    //Measure and calculate tickrate
    tickCounter_++;  
    int64_t dTime;
    if (tickCounterResetInterval_.isTimeToRun(dTime)) {
        tickRate_ = (double)tickCounter_/dTime*SECONDS;
        tickCounter_ = 0;
    }

    if (runPrioGroup(tasks_[0].getChainStart())) return; //Check if a task ran. If so then leave. 
    if (runPrioGroup(tasks_[1].getChainStart())) return;
    if (runPrioGroup(tasks_[2].getChainStart())) return;
    if (runPrioGroup(tasks_[3].getChainStart())) return;
    if (runPrioGroup(tasks_[4].getChainStart())) return;
    if (runPrioGroup(tasks_[5].getChainStart())) return;
    if (runPrioGroup(tasks_[6].getChainStart())) return;

}



bool Scheduler::runPrioGroup(ChainObject<Task>* startOfGroup) {

    ChainObject<Task>* currentTask = startOfGroup;
    bool taskRan = false;

    while(currentTask != nullptr) {
        ChainObject<Task>* nextTask = currentTask->nextObject; // Must be selected before it is removed
        //currentRunningTask_ = &(currentTask->item);
        //if (!currentTask->item.isSuspended) {
        if (NOW() >= currentTask->item.startTime_ns) {
            if (!currentTask->item.initWasCalled) {
                currentTask->item.thread->init();
                currentTask->item.initWasCalled = true;
            }
            if (currentTask->item.timeLimited) {
                if (NOW() - currentTask->item.startTime_ns >= currentTask->item.timeLength_ns) {
                    currentTask->item.thread->removal(); //Run removal function before removing.
                    detachTask(currentTask->item.thread);
                }
            } else if (currentTask->item.interval.isTimeToRun()) {
                currentTask->item.thread->thread();
                taskRan = true;
                if (currentTask->item.numberRunsLeft > 0) {
                    currentTask->item.numberRunsLeft--;
                    if (currentTask->item.numberRunsLeft == 0) {
                        currentTask->item.thread->removal(); //Run removal function before removing.
                        detachTask(currentTask->item.thread);
                    }
                }
            }
        }
        //}
        currentTask = nextTask;

    }

    return taskRan;

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



void Scheduler::attachTask(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority, int64_t startTime_ns, int64_t time_ns) {

    Task task;
    task.thread = function;
    task.interval = IntervalControl(rate_Hz);
    task.startTime_ns = startTime_ns;
    task.timeLength_ns = time_ns;
    //task.interval.setLimit(false);

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

}



void Scheduler::attachTaskForNumberLoops(Thread_Interface* function, uint32_t rate_Hz, eTaskPriority_t priority, uint32_t numberLoops, int64_t startTime_ns) {

    Task task;
    task.thread = function;
    task.interval = IntervalControl(rate_Hz);
    task.startTime_ns = startTime_ns;
    task.numberRunsLeft = numberLoops;
    task.timeLimited = true;

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

}



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
