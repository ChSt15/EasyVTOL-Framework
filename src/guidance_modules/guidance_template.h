#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "templates/module_template.h"

#include "data_containers/kinematic_data.h"



class Guidance: public Module {
private:

    /**
     * Returns a struct containing all kinetic parameters 
     * the vehicle should have to perform its commanded 
     * movements.
     *
     * @param values none.
     * @return Kinetic paramenters.
     */
    virtual KinematicData getKineticSetpoint() = 0;


};





#endif