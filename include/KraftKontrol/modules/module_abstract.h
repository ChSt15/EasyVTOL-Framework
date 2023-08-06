#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



/**
 * This is a template that contains all the functions that all 
 * modules must have.
 * Things like module status und EEPROM are inhereted from this.
*/



#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/modules/communication_modules/kraft_message.h"



enum eModuleStatus_t {
    eModuleStatus_NotStarted,
    eModuleStatus_Starting,
    eModuleStatus_Running,
    eModuleStatus_RestartAttempt,
    eModuleStatus_Failure
};



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
    static Topic<KraftMessage_Interface>& getGlobalMessageTopic() {return globalMessages();}

    /**
     * @returns module specific topic for module communication
     */
    Topic<KraftMessage_Interface>& getModuleMessageTopic() {return moduleMessages_;}

    /**
     * @returns a reference to a list of pointers to all currently existing modules
     */
    static const List<Module_Abstract*>& getListExistingModules() {return existingModules();}


protected:

    Module_Abstract() {existingModules().append(this);}
    virtual ~Module_Abstract() {existingModules().removeAllEqual(this);}

    //This static topic is used for transfering global messages between modules like disarming, failsafe, startup etc.
    static Topic<KraftMessage_Interface>& globalMessages();

    //This topic is used for modules to receive messages from specifically this module.
    Topic<KraftMessage_Interface> moduleMessages_;

    //Every module can use this to signify its current state.
    eModuleStatus_t moduleStatus_ = eModuleStatus_t::eModuleStatus_NotStarted;


private:
    
    static List<Module_Abstract*>& existingModules();

};




#endif