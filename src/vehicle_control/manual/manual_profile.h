#ifndef MANUAL_PROFILE_H
#define MANUAL_PROFILE_H



#include "definitions.h"

#include "utils/interval_control.h"


#define CONTROL_LOOP_RATE 100


class ManualControl {
public:

    void manualControlThread();
    void manualControlInit();

private:

    IntervalControl interval = IntervalControl(CONTROL_LOOP_RATE);

    
};





#endif