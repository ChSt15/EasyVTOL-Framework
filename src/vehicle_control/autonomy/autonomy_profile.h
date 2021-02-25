#ifndef AUTONOMY_PROFILE_H
#define AUTONOMY_PROFILE_H



#include "definitions.h"

#include "utils/interval_control.h"


#define AUTONOMY_LOOP_RATE 100


class Autonomy {
public:

    void autonomyThread();
    void autonomyInit();

private:

    IntervalControl interval = IntervalControl(AUTONOMY_LOOP_RATE);

    
};





#endif