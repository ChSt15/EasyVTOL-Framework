#ifndef GUIDANCE_H
#define GUIDANCE_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "circular_buffer.h"

#include "vehicle/kinetic_data.h"


class Guidance {
public:

    void toPoint(KineticData startState, KineticData endState);
    void toPointLinear(KineticData startState, KineticData endState);

    KineticData getKineticSetpoint() {return _kineticSetpoint;}


protected:

    void guidanceThread();
    

private:

    Circular_Buffer<KineticData, 3> _path; //We will have up to 3 "states" that build the of path the vehicle should follow.

    KineticData _kineticSetpoint;

    
};





#endif