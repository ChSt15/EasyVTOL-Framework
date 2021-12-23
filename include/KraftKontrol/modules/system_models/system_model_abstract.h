#ifndef SYSTEM_MODEL_ABSTRACT_H
#define SYSTEM_MODEL_ABSTRACT_H



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/utils/data_timestamped.h"



/**
 * Abstract/Interface class for System Model classes.
 */
class SystemModel_Abstract {
public:

    /**
     * Uses system model to predict future system state.
     *
     * @param systemState Current or last state that will be used as start for prediction and will be updated with the new predicted state.
     * @param time Delta time to predict. e.g. 100*MILLISECONDS. Must be in nanoseconds.
     * @param iterations Number of iterations. This will split up dTime and run multiple iterations for higher accuracy. Defaults to 1
     */
    virtual void predictState(DataTimestamped<NavigationData>& systemState, int64_t time, uint32_t iterations = 1) = 0;

    /**
     * @returns system mass.
     */
    virtual float getSystemMass() const = 0;

    /**
     * @returns system angular mass.
     */
    virtual Vector<> getSystemAngularMass() const = 0;


};



#endif