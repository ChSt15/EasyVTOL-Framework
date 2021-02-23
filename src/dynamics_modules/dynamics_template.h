#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "templates/module_template.h"

#include "data_containers/kinematic_data.h"
#include "data_containers/dynamic_data.h"



class Dynamics: public Module {
public:

    /**
     * Sets the kinetic setpoints from which the needed forces
     * are calculated.
     *
     * @param values kinematicSetpoint.
     * @return none.
     */
    virtual void setKinematicSetpoint(const KinematicData &kinematicSetpoint) = 0;

    /**
     * Returns the forces the acuators need to produce in order to
     * achieve the kinematic setpoints. 
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual DynamicData getDynamicSetpoint() = 0;
    
    
};





#endif