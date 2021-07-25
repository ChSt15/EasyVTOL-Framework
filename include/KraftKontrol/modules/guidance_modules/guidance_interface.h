#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/utils/topic.h"

#include "KraftKontrol/modules/module_abstract.h"



class Guidance_Interface: public Module_Abstract {
public:

    /**
     * Returns a struct containing all the vehicles
     * setpoint data that feeds to the controller.
     *
     * @return control parameters container.
     */
    ControlData& getControlSetpoint() {return controlSetpoint_;}

    /**
     * @returns reference to guidance modules output topic
     */
    Topic<ControlData>& getControlSetpointTopic() {return controlSetpointTopic_;}


protected:

    ControlData controlSetpoint_;

    Topic<ControlData> controlSetpointTopic_;
    

};





#endif