#ifndef SYSTEM_MODEL_GENERIC_H
#define SYSTEM_MODEL_GENERIC_H



#include "system_model_abstract.h"

#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/actuator_modules/actuator_abstract.h"

#include "KraftKontrol/utils/system_time.h"

#include "lib/MathHelperLibrary/vector_math.h"



/**
 * Generic system model.
 */
class SystemModelGeneric: public SystemModel_Abstract {
private:

    //System mass in kg.
    float systemMass_ = 1;

    //System angular mass.
    Vector<> systemAngularMass_ = 1;


public: 

    /**
     * @param systemMass Mass of the modeled system in kg.
     * @param systemAngularMass Angular mass of the modeled system.
     */
    SystemModelGeneric(float systemMass = 1, Vector<> systemAngularMass = 1);

    /**
     * Uses system model to predict future system state.
     *
     * @param systemState Current or last state that will be used as start for prediction and will be updated with the new predicted state.
     * @param time Absolute time to predict. e.g. NOW() + 100*MILLISECONDS would be 100ms in future from now. Must be in nanoseconds.
     * @param iterations Number of iterations. This will split up dTime and run multiple iterations for higher accuracy. Defaults to 1
     */
    void predictState(DataTimestamped<NavigationData>& systemState, int64_t time, uint32_t iterations = 1) override;

    /**
     * @returns system mass.
     */
    float getSystemMass() const override;

    /**
     * Sets current system mass.
     * @param mass System mass in kg.
     */
    void setSystemMass(float mass);

    /**
     * @returns system angular mass.
     */
    Vector<> getSystemAngularMass() const override;

    /**
     * Sets current system angular mass.
     * @param angularMass System angular mass.
     */
    void setSystemAngularMass(const Vector<>& angularMass);


};



#endif