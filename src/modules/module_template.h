#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



#include "definitions.h"

#include "data_containers/vehicle_data.h"

/**
 * This is a template that contains all the functions that all 
 * modules must have.
*/



enum MODULE_STATUS {
    MODULE_NOT_STARTED,
    MODULE_STARTING,
    MODULE_RUNNING,
    MODULE_RESTARTATTEMPT,
    MODULE_FAILURE
};


inline String deviceStatusToString(MODULE_STATUS status) {

    String buff = "UNKNOWN STATE";

    switch (status)
    {
    case MODULE_STATUS::MODULE_NOT_STARTED :
        buff = "Device not started";
        break;

    case MODULE_STATUS::MODULE_STARTING :
        buff = "Device starting";
        break;

    case MODULE_STATUS::MODULE_RUNNING :
        buff = "Device running";
        break;

    case MODULE_STATUS::MODULE_RESTARTATTEMPT:
        buff = "Device attempting restart";
        break;

    case MODULE_STATUS::MODULE_FAILURE :
        buff = "Device failure";
        break;
    
    default:
        buff = "UNKNOWN STATE: " + String(status);
        break;
    }

    return buff;

}



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
     * Returns how fast the module should be ran at.
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t get_LoopRate_Hz() {
        return 1000;
    }

    /**
     * Returns the modules status
     *
     * @param values none.
     * @return MODULE_STATUS.
     */
    virtual MODULE_STATUS getModuleStatus() {return _moduleStatus;};


protected:

    MODULE_STATUS _moduleStatus = MODULE_STATUS::MODULE_NOT_STARTED;


private:




};




#endif