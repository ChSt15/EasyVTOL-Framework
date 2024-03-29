#ifndef UBLOX_SERIAL_GNSS_H
#define UBLOX_SERIAL_GNSS_H


#include "Arduino.h"

#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_abstract.h"

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/SparkFun_u-blox_GNSS/src/SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "KraftKontrol/utils/buffer.h"



class UbloxSerialGNSS: public GNSS_Abstract, public Module_Abstract, public Task_Threading  {
public:

    /**
     * @param serialPort Pointer to serial port to use. If non default pins used then setup before init run.
     * @param usbPassthrough If true then gps wont be setup and serial data will be passed to USB serial.
     */
    UbloxSerialGNSS(HardwareSerial& serialPort, int rxPin = -1, int txPin = -1) : Task_Threading("Ublox GNSS Serial Driver", eTaskPriority_t::eTaskPriority_High, SECONDS/400) {
        serialPort_ = &serialPort;
        rxPin_ = rxPin;
        txPin_ = txPin;
        usbPassthrough_ = false;// usbPassthrough;
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
     * @returns the Position accuracy. If unsupported or altitude not available will return -1;
     */
    float getPositionAccuracy() {return positionDeviation_;}

    /**
     * @returns the altitude accuracy. If unsupported or altitude not available will return -1;
     */
    float getAltitudeAccuracy() {return altitudeDeviation_;}

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

    void setupSerial(uint32_t baudRate);

    void _getData();

    float positionDeviation_ = -1;
    float altitudeDeviation_ = -1;
    //float velocityDeviation_ = -1;

    int rxPin_;
    int txPin_;

    uint8_t numSats_ = 0;

    //Values used to determine true fix
    uint8_t minNumSats_ = 5;
    float maxError_ = 600;

    bool lockValid_ = false;

    int64_t lastMeasurement_ = 0;

    HardwareSerial* serialPort_;
    //uint32_t serialBaudMulti_ = 1;
    bool usbPassthrough_;

    SFE_UBLOX_GNSS gnss_;

    uint8_t _startAttempts = 0;

    
};






#endif