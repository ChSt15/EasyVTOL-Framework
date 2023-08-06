#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H


#include "clock_abstract.h"

#include "date_time.h"

#include "time_definitions.h"



template<typename TYPE> 
class StaticCallback_Subscriber;


class SystemTime {
private:

    static StaticCallback_Subscriber<DataTimestamped<int64_t>>& clockTopicSubr_();

    static int64_t syncTimeOffset;

    static float timeFactor;


    SystemTime(); //Should be private to prevent class form being created


    static void receiveTime(DataTimestamped<int64_t> const& item);


public:

    /**
     * @brief Get the local time in nanoseconds since system start.
     * 
     * @return int64_t 
     */
    static int64_t getTimeSinceStart();

    /**
     * @brief Get the time synchronised with the given time object. Usually a real time clock set to Unix time.
     * 
     * @return int64_t 
     */
    static int64_t getSyncTime();

    /**
     * @brief Get the TimeDate of when the system started
     * 
     * @return int64_t 
     */
    static int64_t getSystemStartDateTime();

    /**
     * @brief Get the Current Date Time
     * 
     * @return DateTime 
     */
    static TimeDate getCurrentDateTime();

    /**
     * @brief Sets the clock that the system should use to syncronise internal clocks with. 
     * 
     * @param clockSource Clock the the system should sync with
     * 
     */
    static void setClockSource(const Clock_Abstract& clockSource);

    /**
     * @brief Set the Time 
     * 
     * @param timeDate 
     */
    static void setTime(TimeDate timeDate);


};


#endif