#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "modules/templates/module_template.h"

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
    void setKinematicSetpoint(const KinematicData &kinematicSetpoint) {
        _kinematicSetpoint = &_kinematicSetpointDefault;
        *_kinematicSetpoint = kinematicSetpoint;
    };

    /**
     * Sets the kinetic setpoints pointer to the inputed one.
     * 
     * This allows for data linking meaning that the data does 
     * not need to be passed between modules.
     * 
     * Returns false if linking failed. Can only happen if pointer was
     * null pointer.
     *
     * @param values kinematicSetpoint.
     * @return bool.
     */
    bool linkKinematicSetpointPointer(KinematicData* kinematicSetpointPointer) {
        if (kinematicSetpointPointer == nullptr) return false;
        _kinematicSetpoint = kinematicSetpointPointer;
        return true;
    };

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual DynamicData getDynamicSetpoint() {_dynamicOutput;};

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @param values none.
     * @return DynamicData pointer.
     */
    virtual DynamicData* getDynamicSetpointPointer() {&_dynamicOutput;};


protected:

    static KinematicData* _kinematicSetpoint;

    static DynamicData _dynamicOutput;


private:

    static KinematicData _kinematicSetpointDefault;
    
    
};





#endif