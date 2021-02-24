#ifndef GUIDANCE_FLYBYWIRE_H
#define GUIDANCE_FLYBYWIRE_H



#include "guidance_template.h"

#include "vector_math.h"

#include "data_containers/kinematic_data.h"



/**
 * This guidance module is designed to act like a
 * flybywire system. Its a very simple way of
 * transfering manual control to to vehicle.
 */
class GuidanceFlyByWire: public Guidance {
public:

    /**
     * Tells the guidance to rotate vehicle at set rate.
     *
     * @param values rate.
     * @return none.
     */
    void setAngularRate(const Vector &rate) {
        _inputs.angularRate = rate;
    }

    /**
     * Tells the guidance to rotate vehicle to a attitude.
     *
     * @param values attitde.
     * @return none.
     */
    void setAngularRate(const Quaternion &attitude) {
        _inputs.attitude = attitude;
    }

    /**
     * Tells the guidance to move vehicle at a set velocity.
     *
     * @param values attitde.
     * @return none.
     */
    void setVelocity(const Vector &velocity) {
        _inputs.velocity = velocity;
    }

    /**
     * Tells the guidance to move vehicle to a set position.
     *
     * @param values attitde.
     * @return none.
     */
    void setVelocity(const Vector &position) {
        _inputs.position = position;
    }

    void stopVehicle() {
        _inputs.angularRate = Vector(0,0,0);
        _inputs.velocity = Vector(0,0,0);
    }




private:

    KinematicData _inputs;


};





#endif