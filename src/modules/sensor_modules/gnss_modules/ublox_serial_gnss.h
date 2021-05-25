#ifndef UBLOX_SERIAL_GNSS_H
#define UBLOX_SERIAL_GNSS_H


#include "Arduino.h"

#include "data_containers/navigation_data.h"

#include "modules/sensor_modules/gnss_modules/gnss_interface.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "modules/module_abstract.h"

#include "lib/SparkFun_u-blox_GNSS/src/SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "utils/buffer.h"



class UbloxSerialGNSS: public GNSS_Interface, public Module_Abstract, public Task_Abstract  {
public:

    /**
     * @param serialPort Pointer to serial port to use. If non default pins used then setup before init run.
     * @param usbPassthrough If true then gps wont be setup and serial data will be passed to USB serial.
     */
    UbloxSerialGNSS(HardwareSerial* serialPort, bool usbPassthrough = false) : Task_Abstract(100, eTaskPriority_t::eTaskPriority_Realtime, true) {
        serialPort_ = serialPort;
        usbPassthrough_ = usbPassthrough;
    }

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @return uint32_t.
     */
    uint32_t loopRate() {return loopRate_;};

    /**
     * This is only for horizontal position. Even if this is true then that does not mean height is available
     *
     * @returns true if horizontal position available.
     */
    uint16_t positionAvailable() {return positionFifo_.available();};

     /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    uint32_t positionRate() {return positionRate_;}

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param position Struct to be overritten with position data.
     * @returns true if position data valid.
     */
    bool getPosition(WorldPosition* position, uint32_t* positionTimestamp) {

        if (positionFifo_.available() == 0) return false;

        positionFifo_.takeBack(position);
        positionTimestampFifo_.takeBack(positionTimestamp);

        return true;

    };

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param position Struct to be overritten with position data.
     * @returns true if position data valid.
     */
    bool peekPosition(WorldPosition* position, uint32_t* positionTimestamp) {

        if (positionFifo_.available() == 0) return false;

        positionFifo_.peekBack(position);
        positionTimestampFifo_.peekBack(positionTimestamp);

        return true;

    }

    /**
     * @returns the Position accuracy. If unsupported or altitude not available will return -1;
     */
    float getPositionAccuracy() {return positionDeviation_;}

    /**
     * @returns the altitude accuracy. If unsupported or altitude not available will return -1;
     */
    float getAltitudeAccuracy() {return altitudeDeviation_;}

    /**
     * Removes all elements from queue.
     */
    void flushPosition() {
        positionFifo_.clear();
        positionTimestampFifo_.clear();
    }

    /**
     * This is only for horizontal position. Even if this is true then that does not mean height is available
     *
     * @returns true if horizontal position available.
     */
    uint16_t velocityAvailable() {return velocityFifo_.available();}

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @return uint32_t.
     */
    uint32_t velocityRate()  {return positionRate_;}

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param velocity is the velocity.
     * @returns true if position data valid.
     */
    bool getVelocity(Vector<>* velocity, uint32_t* velocityTimestamp) {

        if (velocityFifo_.available() == 0) return false;

        velocityFifo_.takeBack(velocity);
        velocityTimestampFifo_.takeBack(velocityTimestamp);

        return true;

    };

    /**
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param velocity is the velocity.
     * @returns true if position data valid.
     */
    bool peekVelocity(Vector<>* velocity, uint32_t* velocityTimestamp) {

        if (velocityFifo_.available() == 0) return false;

        velocityFifo_.peekBack(velocity);
        velocityTimestampFifo_.peekBack(velocityTimestamp);

        return true;

    }

    /**
     * Removes all elements from queue.
     */
    void flushVelocity() {
        velocityFifo_.clear();
        velocityTimestampFifo_.clear();
    }

    /**
     * @returns the number of satellites used.
     */
    uint8_t getNumSatellites() {return numSats_;}

    /**
     * @returns true if GNSS lock is valid and safe.
     */
    bool getGNSSLockValid() {
        return lockValid_;
    }


private:

    void _getData();


    Buffer <WorldPosition, 10> positionFifo_;
    Buffer <Vector<>, 10> velocityFifo_;
    Buffer <uint32_t, 10> positionTimestampFifo_;
    Buffer <uint32_t, 10> velocityTimestampFifo_;

    float positionDeviation_ = -1;
    float altitudeDeviation_ = -1;
    //float velocityDeviation_ = -1;

    uint8_t numSats_ = 0;

    //Values used to determine true fix
    uint8_t minNumSats_ = 6;
    float maxError_ = 600;

    bool lockValid_ = false;

    IntervalControl rateCalcInterval_ = IntervalControl(1); 
    uint32_t lastMeasurement_ = 0;

    HardwareSerial* serialPort_;
    uint32_t serialBaudMulti_ = 1;
    bool usbPassthrough_;

    SFE_UBLOX_GNSS gnss_;

    uint8_t _startAttempts = 0;

    uint32_t loopRate_ = 0;
    uint32_t loopCounter_ = 0;

    uint32_t positionRate_ = 0;
    uint32_t positionCounter_ = 0;

    uint32_t velocityRate_ = 0;
    uint32_t velocityCounter_ = 0;

    bool _block = false;

    
};






#endif