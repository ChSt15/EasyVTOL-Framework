#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"



/**
 * ToDo:
 * - Place tasks sorted into list. This way we dont have to search for next lower priority, only go down the list.
 * - Give lower priotrity tasks a chance to run if they havent ran for a long time (> 10 times interval or something proportional to priority?...).
 * - More fixes?
 */



//List<Task_Abstract*> Task_Abstract::taskList();
IntervalControl Task_Abstract::systemResourceCalcInterval_ = 1;
float Task_Abstract::schedulerUsage_;



void Task_Abstract::schedulerInitTasks() {

    for (uint32_t i = 0; i < taskList().getNumItems(); i++) {
        taskList()[i]->init();
        taskList()[i]->initWasCalled_ = true;
    }

}


void Task_Abstract::schedulerTick() {

    uint32_t lastPriority = UINT32_MAX;
    uint32_t currentPriority = UINT32_MAX;

    bool taskRan = false;
    bool taskFound = true;

    Task_Abstract* task;


    while (!taskRan && taskFound) {

        //Check if any tasks with this priority need to be ran
        for (uint32_t i = 0; i < taskList().getNumItems(); i++) {

            task = taskList()[i];

            //Only run a task if its not suspended and priority is same as one to run.
            if (!task->isSuspended_ && task->priority_ == currentPriority && NOW() >= task->startTime_) {
                if (!task->initWasCalled_) {
                    task->init();
                    task->initWasCalled_ = true;
                }
                if (NOW() >= task->endTime_) {
                    task->removal(); //Run removal function before removing.
                    task->stopTaskThreading();
                }
                if (task->interval_.isTimeToRun()) {
                    task->run();
                    taskRan = true;
                    if (task->removeOnRun > 0) {
                        task->numberRuns_++;
                        if (task->numberRuns_ >= task->removeOnRun) {
                            task->removal(); //Run removal function before removing.
                            task->stopTaskThreading();
                        }
                    }
                }
            }

        }

        
        //Find the next lower highest priority in task list
        if (!taskRan) {

            lastPriority = currentPriority;
            currentPriority = 0;

            taskFound = false;

            for (uint32_t i = 0; i < taskList().getNumItems(); i++) {

                task = taskList()[i];

                if (!task->isSuspended_ && task->priority_ > currentPriority && task->priority_ < lastPriority) {
                    currentPriority = task->priority_;
                    taskFound = true;
                }

            }

        }

    }


    int64_t dTime = 0;
    if (systemResourceCalcInterval_.isTimeToRun(dTime)) {

        //Calculate time usage for each task.
        int64_t totalTime = dTime;  
        int64_t timeLeft = totalTime;
        for (uint32_t i = 0; i < taskList().getNumItems(); i++) {

            task = taskList()[i];

            task->systemUsage_ = (float)task->systemTimeUsageCounter_/totalTime;
            timeLeft -= task->systemTimeUsageCounter_;
            task->systemTimeUsageCounter_ = 0;

            task->runRate_ = task->runCounter_/((double)dTime/SECONDS);
            task->runCounter_ = 0;

        }

        schedulerUsage_ = (float)timeLeft/totalTime;

    }


}



List<Task_Abstract*>& Task_Abstract::taskList() {

    static List<Task_Abstract*> taskList_g = List<Task_Abstract*>();

    return taskList_g;

}