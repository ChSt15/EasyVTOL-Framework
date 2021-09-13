#ifndef MODULE_AUTORUN_CLASS_H
#define MODULE_AUTORUN_CLASS_H



//#include "simple_scheduler.h"

#include "KraftKontrol/utils/chain_buffer.h"
#include "interval_control.h"

#include "../list.h"



/**
 * Gives a task a priority. 
 * Lower priorities only run if no tasks exist at higher priorities.
 * Tasks with equal priorities will run one after another.
 * Tasks with REALTIME will always run. But must be written to run fast as to give
 * lower priority tasks time to run.
 */ 
enum eTaskPriority_t: uint32_t {
    //Will only run if there is nothing to do.
    eTaskPriority_None = 0,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_VeryLow = 1,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_Low = 2,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_Middle = 3,
    //Will only run if nothing to do at higher priorities.
    eTaskPriority_High = 4,
    //Will only run if nothing to do at higher priorities. Recommended for I2C comms!
    eTaskPriority_VeryHigh = 5,
    //Will always run once it needs to. Good for devices over SPI. I2C might be too slow use eTaskPriority_VeryHigh!
    eTaskPriority_Realtime = UINT32_MAX,
};


class Task_Abstract {
private:

    //List containing all tasks
    //static List<Task_Abstract*> taskList_;

    //Interval control for calculating system resource usage.
    static IntervalControl systemResourceCalcInterval_;

    static float schedulerUsage_;

    static List<Task_Abstract*>& taskList();

    
    //Local variables each task has

    //Task name, up to 30 characters long
    char name_[30];

    //Controls task interval
    IntervalControl interval_;

    //If true then do not run task.
    bool isSuspended_ = false;

    //To store whether init function was ran.
    bool initWasCalled_ = false;

    //When to remove task.
    int64_t endTime_ = END_OF_TIME;
    //When to start running task. 
    int64_t startTime_ = 0;

    //If set to 0 then no limit.
    uint64_t removeOnRun = 0;
    uint64_t numberRuns_ = 0;

    //Task Priority
    uint32_t priority_ = eTaskPriority_t::eTaskPriority_None;

    //This should be overloaded by task subclass
    virtual void thread() {stopTaskThreading();};

    //Counter for number of runs
    uint32_t runCounter_ = 0;

    //Timestamp for last run counter reset
    int64_t lastCounterTimestamp_ = 0;

    //Rate at which run is being called
    uint32_t runRate_ = 0;

    //Timestamps used to find out task time usage.
    float systemUsage_ = 0;
    int64_t systemTimeUsageCounter_ = 0;


    void addTaskToScheduler();
    void removeTaskFromScheduler();

    
public:

    /**
     * Sets up the task and attaches it to the internal scheduler.
     * 
     * @param taskName Char array containing name of task. Max 30 characters long.
     * @param rate is the rate at which to run the Task
     * @param priority is of type uint32_t and gives the priority of the task. Higher number is higher priority.
     * @param startRunning will auto start threading if set to true. Default is true.
     * @param runs sets the number of times to run the thread function. Set to -1 for infinite. Default is -1.
     */
    Task_Abstract(const char* taskName, uint32_t rate, uint32_t priority, int64_t startTime = 0, int64_t endTime = END_OF_TIME, uint64_t numberRuns = 0) {
        strncpy(name_, taskName, 20);
        interval_ = rate;
        priority_ = priority;
        startTime_ = startTime;
        endTime_ = endTime;
        numberRuns_ = numberRuns;
        isSuspended_ = false;
        taskList().removeAllEqual(this); //Make sure task isnt already in list.
        taskList().append(this);
    }

    /**
     * Will remove Task from scheduler.
     */
    virtual ~Task_Abstract() {
        taskList().removeAllEqual(this);
    }

    /**
     * Starts the threading for the task.
     * 
     * @param numberRuns sets the number of times to run the thread function. Set to -1 for infinite. Default is -1.
     * @returns true if added or false if rate is set to 0 or scheduler failed to add task.
     */
    bool startTaskThreading() {
        isSuspended_ = false;
        return true;
    }

    /**
     * Stops threading for task.
     */
    void stopTaskThreading() {
        isSuspended_ = true;
    }

    /**
     * Sets task rate to run at.
     * 
     * @param rate is the rate to run at.
     */
    void setTaskRate(uint32_t rate) {interval_.setRate(rate);}

    /**
     * Sets task priority.
     * 
     * @param priority is of type uint32_t.
     */
    void setTaskPriority(uint32_t priority) {priority_ = priority;}

    /**
     * Returns task priority.
     * 
     * @returns priority of type eTaskPriority_t.
     */
    uint32_t getTaskPriority() {
        return priority_;
    }

    /**
     * Returns task rate.
     * 
     * @returns rate of type uint32_t.
     */
    uint32_t getTaskRate() {
        return interval_.getRate();
    }

    /**
     * @returns amount of system usage task takes in percent.
     */
    float getTaskSystemUsage() {
        return systemUsage_;
    }

    const char* getTaskName() {
        return name_;
    }

    /**
     * @returns amount of system usage internal scheduler takes in percent.
     */
    static float getSchedulerSystemUsage() {
        return schedulerUsage_;
    }

    /**
     * Static function to give internal scheduler time to run tasks.
     * This needs to be ran as often and fast as possible to give all tasks time.
     * 
     * Usually the main program loop only constantly calls this.
     * Warning: Do not call this inside a task! Can cause stack overflow!
     */
    static void schedulerTick();

    /**
     * Static function that tells scheduler to initialize all attached tasks.
     */
    static void schedulerInitTasks();

    /**
     * @returns list containing pointers to all tasks.
     */
    static List<Task_Abstract*>& getTaskList() {
        return taskList();
    }

    /**
     * Used to get how often per second tick() from the scheduler is called. Can be used to see how the systems performance is.
     * 
     * @returns tick rate.
     */
    //static uint32_t getSchedulerTickRate() {return g_scheduler.getTickRate();}


    //Is called by scheduler
    void run() {

        runCounter_++;

        int64_t start = NOW();
        thread();
        systemTimeUsageCounter_ += NOW() - start;

    }


    //Below are functions to be implemented by subclasses.


    //This is ran once before first run of thread()
    virtual void init() {};

    //This is ran only when the Task is removed from scheduler.
    virtual void removal() {};

    /**
     * @returns the rate at which the task is being called at.
     */
    uint32_t getLoopRate() {return runRate_;}


protected:

    /**
     * Calling this will pause the thread until the given time was reached.
     * e.g. waitUntil(NOW() + 100*MILLISECONDS) will delay the program for 100 milliseconds.
     * 
     * This might not be accurate if many threads suspend themselves. Only one thread should ever call this.
     * 
     * @param time Time in nanoseconds at which the thread should continue
     */
    void suspendUntil(int64_t time) {

        stopTaskThreading();
        while(NOW() < time) schedulerTick();
        startTaskThreading();
        
    }

};



#endif
