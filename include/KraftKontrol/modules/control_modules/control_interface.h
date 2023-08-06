#ifndef CONTROL_ABSTRACT_H
#define CONTROL_ABSTRACT_H



#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "KraftKontrol/utils/data_timestamped.h"



class Control_Interface: public Module_Abstract {
public:

    /**
     * Returns the kinematics the system needs to achieve the desired
     * guidance inputs.
     *
     * @return dynamicSetpoint.
     */
    DataTimestamped<DynamicData>& getDynamicsOutput() {return controlOutput_;};

    /**
     * @returns topic for control output.
     */
    Topic<DataTimestamped<DynamicData>>& getControlDataTopic() {return controlTopic_;};

    /**
     * Resets control module. Things like PID I values are set to 0.
     */
    virtual void reset() = 0;


protected:

    Control_Interface() {}

    //Newest output data.
    DataTimestamped<DynamicData> controlOutput_;

    //Topic for distributing new control data.
    Topic<DataTimestamped<DynamicData>> controlTopic_;

};





#endif