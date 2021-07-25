#ifndef INTERVAL_CONTROL_H
#define INTERVAL_CONTROL_H



#include "KraftKontrol/utils/system_time.h"



class IntervalControl {
public:

    IntervalControl() {
        lastRun_ns_ = 0;
    }

    IntervalControl(float rate) {
        lastRun_ns_ = 0;
        if (rate > 0.0f) setRate(rate);
    }

    /**
     * Sets the rate in Hz
     *
     * @param values rate in Hz
     * @return none.
     */
    inline void setRate(float rate_Hz) {interval_ns_ = (int64_t)SECONDS/rate_Hz;}

    /**
     * Gets the rate in Hz
     *
     * @param values none.
     * @return uint32_t.
     */
    inline uint32_t getRate() {return SECONDS/interval_ns_;}

    /**
     * Sets the interval in milliseconds
     *
     * @param values interval in milliseconds
     * @return none.
     */
    inline void setInterval(int64_t interval_ns) {interval_ns_ = interval_ns; lastRun_ns_ = NOW() - interval_ns_;}

    /**
     * Gets the interval in milliseconds
     *
     * @param values none.
     * @return interval in milliseconds.
     */
    inline int64_t getInterval() {return interval_ns_;}

    /**
     * Returns the amount of time till the next run.
     * Negative number means the how long ago it should have ran
     *
     * @param values none
     * @return remaining time till next run.
     */
    inline int64_t getTimeRemain() {return -interval_ns_ + NOW() + lastRun_ns_;}

    /**
     * Sets the way the system limits the runs.
     * If limit is true then this will only limit the rate
     * and if the run takes longer than the set interval the 
     * system will not compensate for the lost time.
     * If limit is false then if a run is late the system will make
     * the next runs come sooner. This causes abnormal intervals but
     * is usefull if somthing has to run a certain amount of times 
     * per second.
     *
     * @param values limit
     * @return none.
     */
    inline void setLimit(bool limit) {limit_ = limit;}

    /**
     * This syncs the internal variables. This is usefull
     * if the limiter is off and to keep for huge amount of 
     * lost time to keep the system from running at max speed.
     * e.g. if say rate is set to 1khz and does not run for 1s
     * and syncRate() is not called then system will make up 
     * for the 1000 lost runs by running 1000 times as fast
     * as possible.
     *
     * @param values none
     * @return none.
     */
    inline void syncInternal() {lastRun_ns_ = NOW();}

    /**
     * Waits till next Run.
     * See overload to use this time for other functions
     *
     * @param values none
     * @return none.
     */
    inline void waitTillNextRun() {

        while(NOW() - lastRun_ns_ < interval_ns_ || block_);
        
        if (limit_) lastRun_ns_ = NOW();
        else lastRun_ns_ += interval_ns_;

    }

    /**
     * Waits till next Run but calls the given
     * method in the time it waits
     *
     * @param values method to run while waiting
     * @return none.
     */
    inline void waitTillNextRun(void (*callMethod)(void)) {

        while (NOW() - lastRun_ns_ < interval_ns_ || block_) callMethod();
        
        if (limit_) lastRun_ns_ = NOW();
        else lastRun_ns_ = NOW() - (NOW()%interval_ns_);

    }

    /**
     * Checks if it is time to run and returns
     * true if its time to run. 
     * If updateClock is true then the internal interval
     * is set for next run. If you only want to check if its
     * time to run then set updateClock to false;
     * Usefull for if statements. 
     * e.g:
     * 
     * if (x.isTimeToRun()) {
     *      Stuff to do periodically...
     * }
     *
     * @param values updateClock
     * @return none.
     */
    inline bool isTimeToRun(bool updateClock = true) {

        if (block_) return false;
        
        if (NOW() - lastRun_ns_ > interval_ns_) {
            if (!limit_ && updateClock) lastRun_ns_ = NOW() - (NOW()%interval_ns_);//{while(lastRun_ns_ + interval_ns_ < NOW()) lastRun_ns_ += interval_ns_;}
            else if (updateClock) lastRun_ns_ = NOW();
            return true;
        } else return false;

    }

    /**
     * Checks if it is time to run and returns
     * true if its time to run. 
     * If updateClock is true then the internal interval
     * is set for next run. If you only want to check if its
     * run to run then set updateClock to false;
     * Usefull for if statements. 
     * This overload will write the time delta from the 
     * last run into the timeDelta variable. This way
     * the time from the last run is passed on.
     * e.g:
     * 
     * if (x.isTimeToRun()) {
     *      Stuff to do periodically...
     * }
     *
     * @param values updateClock
     * @return none.
     */
    inline bool isTimeToRun(int64_t& timeDelta, bool updateClock = true) {

        if (block_) return false;
        
        timeDelta = NOW() - lastRun_ns_;
        if (timeDelta > interval_ns_) {
            if (!limit_ && updateClock) lastRun_ns_ = NOW() - (NOW()%interval_ns_);
            else if (updateClock) lastRun_ns_ = NOW();
            return true;
        } else return false;

    }

    /**
     * If block is enabled then .isTimeToRun() will
     * always return false and .waitTillNextRun() will
     * never exit. This can be undone by calling the 
     * method and giving false as a parameter.
     *
     * @param values block
     * @return none.
     */
    inline void block(bool block) {
        block_ = block;
    }


private:

    int64_t lastRun_ns_ = 0;

    int64_t interval_ns_ = 0;

    bool limit_ = false;

    bool block_ = false;

};





#endif