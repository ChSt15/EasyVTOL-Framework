#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



/**
 * This is a template that contains all the functions that all 
 * modules must have.
 * Things like module status und EEPROM are inhereted from this.
*/



#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/KraftKommunikation/kraft_message.h"



enum eModuleStatus_t {
    eModuleStatus_NotStarted,
    eModuleStatus_Starting,
    eModuleStatus_Running,
    eModuleStatus_RestartAttempt,
    eModuleStatus_Failure
};


#ifdef ARDUINO_H

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

#endif


/**
 * Although not here declared, void thread() must be defined in the derived class and will be automatically ran by scheduler.
 */
class Module_Abstract {
public:

    /**
     * Returns the modules status
     *
     * @param values none.
     * @return MODULE_STATUS.
     */
    virtual eModuleStatus_t getModuleStatus() {return moduleStatus_;}

    /**
     * @returns global topic for module communication
     */
    static Topic<KraftMessageContainer>& getGlobalMessageTopic() {return globalMessages_;}


protected:

    //This static topic is used for transfering global messages between modules like disarming, failsafe, startup etc.
    static Topic<KraftMessageContainer> globalMessages_;

    //Every module can use this to signify its current state.
    eModuleStatus_t moduleStatus_ = eModuleStatus_t::eModuleStatus_NotStarted;


};




#endif