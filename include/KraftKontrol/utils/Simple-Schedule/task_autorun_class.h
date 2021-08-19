#ifndef MODULE_AUTORUN_CLASS_H
#define MODULE_AUTORUN_CLASS_H



#include "simple_scheduler.h"



namespace {

    Scheduler g_scheduler;

}



class Task_Abstract: public Thread_Interface {
public:

    /**
     * Sets up the task and attaches it to the internal scheduler.
     * 
     * @param rate is the rate at which to run the Task
     * @param priority is of type eTaskPriority_t and gives the priority of the task
     * @param startRunning will auto start threading if set to true. Default is true.
     * @param runs sets the number of times to run the thread function. Set to -1 for infinite. Default is -1.
     */
    Task_Abstract(uint32_t rate, eTaskPriority_t priority, bool startRunning = false) {
        rate_ = rate;
        priority_ = priority;
        if (startRunning) startTaskThreading();
    }

    /**
     * Will remove Task from scheduler.
     */
    ~Task_Abstract() {
        g_scheduler.detachTask(this);
    }

    /**
     * Starts the threading for the task.
     * 
     * @param numberRuns sets the number of times to run the thread function. Set to -1 for infinite. Default is -1.
     * @returns true if added or false if rate is set to 0 or scheduler failed to add task.
     */
    bool startTaskThreading() {
        if (rate_ == 0) return false;
        if (attached_) return true; //Keep from attaching itsself multiple times
        g_scheduler.attachTask(this, rate_, priority_);
        attached_ = true;
        return true;
    }

    /**
     * Stops threading for task.
     */
    void stopTaskThreading() {
        if (!attached_) return; //Dont need to remove itsself if not attached
        g_scheduler.detachTask(this);
        attached_ = false;
    }

    /**
     * Sets task rate to run at.
     * 
     * @param rate is the rate to run at.
     */
    void setTaskRate(const uint32_t &rate) {rate_ = rate;}

    /**
     * Sets task priority.
     * 
     * @param priority is of type eTaskPriority_t.
     */
    void setTaskPriority(const eTaskPriority_t &priority) {priority_ = priority;}

    /**
     * Returns task priority.
     * 
     * @returns priority of type eTaskPriority_t.
     */
    eTaskPriority_t getTaskPriority() {
        return priority_;
    }

    /**
     * Returns task rate.
     * 
     * @returns rate of type uint32_t.
     */
    uint32_t getTaskRate() {
        return rate_;
    }

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
        g_scheduler.suspendUntil(timeInMicrosecond);
    }*/

    /**
     * Static function to give internal scheduler time to run tasks.
     * This needs to be ran as often and fast as possible to give all tasks time.
     * 
     * Usually the main program loop only constantly calls this.
     * Warning: Do not call this inside a task! Can cause stack overflow!
     */
    static void schedulerTick() {g_scheduler.tick();}

    /**
     * Static function that tells scheduler to initialize all attached tasks.
     */
    static void schedulerInitTasks() {g_scheduler.initializeTasks();}

    /**
     * Used to get how often per second tick() from the scheduler is called. Can be used to see how the systems performance is.
     * 
     * @returns tick rate.
     */
    static uint32_t getSchedulerTickRate() {return g_scheduler.getTickRate();}

    /**
     * Defined now but can be overridden. This way is does not need to be defined by user.
     */
    virtual void removal() {}

    /**
     * Defined now but can be overridden. This way is does not need to be defined by user.
     */
    virtual void init() {}


private:

    uint32_t rate_ = 0;
    eTaskPriority_t priority_ = eTaskPriority_t::eTaskPriority_None;
    bool attached_ = false;

};



#endif
