#ifndef QMC5883_DRIVER_TEMPLATE_H
#define QMC5883_DRIVER_TEMPLATE_H



#include "Arduino.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "modules/bus_hal_modules/arduino_i2c_bus_hal.h"

#include "modules/eeprom_hal_modules/eeprom_hal_interface.h"
#include "KraftPacket_KontrolPackets/kraftkontrol_message_types.h"

#include "modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"

#include "modules/module_abstract.h"

#include "utils/buffer.h"



namespace QMC5883Registers {

    const uint8_t QMC5883L_ADDR_DEFAULT = 0x0D;

    const uint8_t QMC5883L_X_LSB = 0;
    const uint8_t QMC5883L_X_MSB = 1;
    const uint8_t QMC5883L_Y_LSB = 2;
    const uint8_t QMC5883L_Y_MSB = 3;
    const uint8_t QMC5883L_Z_LSB = 4;
    const uint8_t QMC5883L_Z_MSB = 5;
    const uint8_t QMC5883L_STATUS = 6;
    const uint8_t QMC5883L_TEMP_LSB = 7;
    const uint8_t QMC5883L_TEMP_MSB = 8;
    const uint8_t QMC5883L_CONFIG = 9;
    const uint8_t QMC5883L_CONFIG2 = 10;
    const uint8_t QMC5883L_RESET = 11;
    const uint8_t QMC5883L_RESERVED = 12;
    const uint8_t QMC5883L_CHIP_ID = 13;


}



class QMC5883Driver: public Magnetometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    /**
     * This is where all calculations are done.
     *
     * @param bus Pointer to I2C bus to use.
     * @param address Address of QMC5883L. Default 0x0D.
     * @param eeprom Pointer to EEPROM module to use for calibration values.
     */
    QMC5883Driver(TwoWire* bus, uint8_t address = QMC5883Registers::QMC5883L_ADDR_DEFAULT, EEPROM_Interface* eeprom = nullptr) : Task_Abstract(250, eTaskPriority_t::eTaskPriority_VeryHigh, true), bus_(bus, QMC5883Registers::QMC5883L_ADDR_DEFAULT) {
        eeprom_ = eeprom;
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
     * @param values none.
     * @return uint32_t.
     */
    uint32_t loopRate() {return _loopRate;};

    /**
     * Returns true if Magnetometer data available
     *
     * @param values none.
     * @return bool.
     */
    uint32_t magAvailable() {return magFifo_.available();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t magRate() {return _magRate;};

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool getMag(Vector<>* magData, uint32_t* magTimestamp) {

        if (magFifo_.available() == 0) return false;

        magFifo_.takeBack(magData);
        magTimestampFifo_.takeBack(magTimestamp);

        return true;

    };

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool peekMag(Vector<>* magData, uint32_t* magTimestamp) {

        if (magFifo_.available() == 0) return false;

        magFifo_.peekBack(magData);
        magTimestampFifo_.peekBack(magTimestamp);

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushMag() {
        magFifo_.clear();
        magTimestampFifo_.clear();
    }

    /**
     * @returns current calibration status
     */
    eMagCalibStatus_t getCalibrationStatus() {return calibrationStatus_;}

    /**
     * Starts calibration sequence.
     */
    void startCalibration() {calibrate_ = true; calibrationStart_ = micros();}

    /**
     * Stops calibration sequence.
     */
    void stopCalibration() {calibrate_ = false;}


private:

    void getData();

    bool dataAvailable();

    bool getEEPROMData();
    bool setEEPROMData();


    Buffer <Vector<>, 10> magFifo_;
    Buffer <uint32_t, 10> magTimestampFifo_;

    Vector<> _lastMag;

    EEPROM_Interface* eeprom_ = nullptr;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    I2CBus_HAL bus_;

    uint8_t _startAttempts = 0;

    uint32_t _loopRate = 0;
    uint32_t _loopCounter = 0;

    uint32_t _magRate = 0;
    uint32_t magCounter_ = 0;

    uint32_t _lastMeasurement = 0;

    Vector<> magMin_ = -1;
    Vector<> magMax_ = 1;

    bool _block = false;

    bool calibrate_ = false;
    uint32_t calibrationStart_ = 0;

    eMagCalibStatus_t calibrationStatus_ = eMagCalibStatus_t::eMagCalibStatus_NotCalibrated;

    
};



#endif