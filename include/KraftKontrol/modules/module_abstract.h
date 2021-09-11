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
 * Modules inhereting from this class automatically support module status and module topics for communication.
 */
class Module_Abstract {
public:

    /**
     * Returns the modules status
     *
     * @param values none.
     * @return MODULE_STATUS.
     */
    eModuleStatus_t getModuleStatus() {return moduleStatus_;}

    /**
     * @returns global topic for module communication
     */
    static Topic<KraftMessageContainer>& getGlobalMessageTopic() {return globalMessages();}

    /**
     * @returns module specific topic for module communication
     */
    Topic<KraftMessageContainer>& getModuleMessageTopic() {return moduleMessages_;}

    /**
     * @returns a reference to a list of pointers to all currently existing modules
     */
    static const List<Module_Abstract*>& getListExistingModules() {return existingModules();}


protected:

    Module_Abstract() {existingModules().append(this);}
    virtual ~Module_Abstract() {existingModules().removeAllEqual(this);}

    //This static topic is used for transfering global messages between modules like disarming, failsafe, startup etc.
    static Topic<KraftMessageContainer>& globalMessages();
    //This static topic is used for distributing received telemetry messages
    static Topic<KraftMessageContainer>& telemetryMessages();

    //This topic is used for modules to receive messages from specifically this module.
    Topic<KraftMessageContainer> moduleMessages_;

    //Every module can use this to signify its current state.
    eModuleStatus_t moduleStatus_ = eModuleStatus_t::eModuleStatus_NotStarted;


private:
    
    static List<Module_Abstract*>& existingModules();

};




#endif