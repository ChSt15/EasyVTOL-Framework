#include "scheduler.h"



/**
 * Scheduler.
 *
 * @param values none.
 * @return none.
 */
void Scheduler::initialise() {

}

/**
 * This is the loop for the scheduler.
 * It will check for functions that need to be ran and run them.
 *
 * @param values none.
 * @return none.
 */
void Scheduler::tick() {

    //Run all tasks with a priority. If none are found then go to next lower priority.

    ChainObject<Task>* _currentTask = _tasks[0].getChainStart();
    bool taskRan = false; //Set to true if a task was run. Will only be checked if we switch to lower task.

    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[1].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[2].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[3].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[4].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[5].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }

    if (taskRan) return; //Check if a task ran. If so then leave.
    _currentTask = _tasks[6].getChainStart(); //Switch to next priority
    while(_currentTask != nullptr) {
        if (_currentTask->item.interval.isTimeToRun() && _currentTask->item.continualRun) {
            _currentTask->item.function();
            taskRan = true;
        }
        if (_currentTask->item.limited) {
            if (micros() - _currentTask->item.creationTimestamp_us >= _currentTask->item.removeThreshold_us) {
                _currentTask->item.function();
                detachFunction(_currentTask->item.function);
                taskRan = true;
            }
        }
        _currentTask = _currentTask->nextObject;
    }
    

}

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
bool Scheduler::attachFunction(void (*function)(void), uint32_t rate_Hz, TASK_PRIORITY priority = TASK_PRIORITY::PRIORITY_MIDDLE) {

    Task task;
    task.function = function;
    task.interval = IntervalControl(rate_Hz);

    switch (priority) {

    case TASK_PRIORITY::PRIORITY_REALTIME:
        _tasks[0].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_VERYHIGH :
        _tasks[1].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_HIGH :
        _tasks[2].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_MIDDLE :
        _tasks[3].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_LOW :
        _tasks[4].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_VERYLOW :
        _tasks[5].addItem(task);
        break;
    
    default:
        _tasks[6].addItem(task);
        break;
    }

    return true;
}

/**
 * The given function will be ran at the given time.
 * 
 * 
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
bool Scheduler::attachFunction(void (*function)(void), uint32_t rate_Hz, TASK_PRIORITY priority, uint32_t time_us, bool continualRun) {

    Task task;
    task.function = function;
    task.interval = IntervalControl(rate_Hz);
    task.creationTimestamp_us = micros();
    task.removeThreshold_us = time_us;
    task.continualRun = continualRun;
    task.limited = true;

    switch (priority) {

    case TASK_PRIORITY::PRIORITY_REALTIME:
        _tasks[0].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_VERYHIGH :
        _tasks[1].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_HIGH :
        _tasks[2].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_MIDDLE :
        _tasks[3].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_LOW :
        _tasks[4].addItem(task);
        break;

    case TASK_PRIORITY::PRIORITY_VERYLOW :
        _tasks[5].addItem(task);
        break;
    
    default:
        _tasks[6].addItem(task);
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
bool Scheduler::detachFunction(void (*function)(void)) {

    Task toBeRemoved;
    toBeRemoved.function = function;

    for (uint8_t i = 0; i < 7; i++) {
        if (_tasks[i].removeItem(toBeRemoved)) {
            return true;
        }
    }

    return false;
}
