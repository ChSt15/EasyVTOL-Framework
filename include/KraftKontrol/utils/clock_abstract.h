#ifndef CLOCK_ABSTRACT_H
#define CLOCK_ABSTRACT_H


#include "data_timestamped.h"

#include "topic.h"


class Clock_Abstract {
private:


protected:

    Topic<DataTimestamped<int64_t>> clockTopic_;


public:

    const Topic<DataTimestamped<int64_t>>& getClockTopic() const {return clockTopic_;}


};


#endif