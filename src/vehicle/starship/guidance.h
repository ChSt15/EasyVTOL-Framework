#ifndef GUIDANCE_H
#define GUIDANCE_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "circular_buffer.h"

#include "vehicle/kinetic_data.h"


class Guidance {
public:

    /**
     * Tells to vehicle to go from one point to another.
     * The path that the vehicle should take might not be 
     * a line. For that use toPointLinear().
     * This program also takes the velocities, and attitudes
     * of the start and end point.
     * Some vehicles might not support (ignore) certain
     * Kinetic parameters. E.g a multicopter due to it being 
     * underactuated will ignore attitude parameters. 
     *
     * @param values startPoint and endpoint.
     * @return none.
     */
    void toPoint(KineticData startState, KineticData endState);

    /**
     * Exacly the same as toPoint() (see for further infos) but 
     * interpolates the parameters to follow a line.
     *
     * @param values startPoint and endpoint.
     * @return none.
     */
    void toPointLinear(KineticData startState, KineticData endState);

    /**
     * Tells to vehicle to go to a point that is relative
     * to the vehicle SETpoint not ISpoint. This also takes
     * the vehicles orientation into account. Meaning moving
     * 5 meters to the left is from the vehicles left.
     * The path that the vehicle should take might not be 
     * a line. For that use toPointLinearREL().
     * This program also takes the velocities, and attitudes
     * of the start and end point.
     * Some vehicles might not support (ignore) certain
     * Kinetic parameters. E.g a multicopter due to it being 
     * underactuated will ignore attitude parameters. 
     *
     * @param values only endpoint.
     * @return none.
     */
    void toPointREL(KineticData endState);

    /**
     * Exacly the same as toPointRel() (see for further infos) but 
     * interpolates the parameters to follow a line.
     *
     * @param values only endpoint.
     * @return none.
     */
    void toPointLinearREL(KineticData endState);

    /**
     * Returns the kinetic setpoints the vehicle has to follow
     * to achieve the commands.
     *
     * @param values none.
     * @return kinetic setpoint.
     */
    KineticData getKineticSetpoint() {return _kineticSetpoint;}


protected:

    void guidanceThread();
    

private:

    Circular_Buffer<KineticData, 3> _path; //We will have up to 3 "states" that build the of path the vehicle should follow.

    KineticData _kineticSetpoint;

    
};





#endif