#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "templates/module_template.h"

#include "data_containers/kinematic_data.h"


class Control: public Module {
public:

    /**
     * Sets the kinetic setpoints from the guidance module.
     *
     * @param values kinematicSetpoint.
     * @return none.
     */
    virtual void setKinematicSetpoint(const KinematicData &kinematicSetpoint) = 0;

    /**
     * Returns the kinematics the system needs to achieve the set kinematics
     *
     * @param values none.
     * @return kinematicSetpoint.
     */
    virtual KinematicData getKinematicSetpoint() = 0;
    
    
};





#endif