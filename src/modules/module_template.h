#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



#include "module_autorun_class.h"



/**
 * This is a template that contains all the functions that all 
 * modules must have.
*/



enum eModuleStatus_t {
    eModuleStatus_NotStarted,
    eModuleStatus_Starting,
    eModuleStatus_Running,
    eModuleStatus_RestartAttempt,
    eModuleStatus_Failure
};


inline String deviceStatusToString(eModuleStatus_t status) {

    String buff = "UNKNOWN STATE";

    switch (status)
    {
    case eModuleStatus_t::eModuleStatus_NotStarted :
        buff = "Device not started";
        break;

    case eModuleStatus_t::eModuleStatus_Starting :
        buff = "Device starting";
        break;

    case eModuleStatus_t::eModuleStatus_Running :
        buff = "Device running";
        break;

    case eModuleStatus_t::eModuleStatus_RestartAttempt :
        buff = "Device attempting restart";
        break;

    case eModuleStatus_t::eModuleStatus_Failure :
        buff = "Device failure";
        break;
    
    default:
        buff = "UNKNOWN STATE: " + String(status);
        break;
    }

    return buff;

}


/**
 * Although not here declared, void thread() must be defined in the derived class and will be automatically ran by scheduler.
 */
class Module_Abstract: public ModuleAutoRun_Abstract {
public:

    /**
     * Creates a module based class and automatically adds it to the scheduler.
     * 
     * @param rate is the rate at which it will be ran at.
     * @param priority is the priority the module will have.
     */
    Module_Abstract(uint32_t rate, eTaskPriority_t priority) : ModuleAutoRun_Abstract(rate, priority) {}

    /**
     * Module specific init function that sets the 
     * module up.
     *
     * @param values none.
     * @return none.
     */
    virtual void init() = 0;

    /**
     * Returns the modules status
     *
     * @param values none.
     * @return MODULE_STATUS.
     */
    virtual eModuleStatus_t getModuleStatus() {return _moduleStatus;};


protected:

    eModuleStatus_t _moduleStatus = eModuleStatus_t::eModuleStatus_NotStarted;


};




#endif