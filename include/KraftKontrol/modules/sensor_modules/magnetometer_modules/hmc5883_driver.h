#ifndef QMC5883_DRIVER_TEMPLATE_H
#define QMC5883_DRIVER_TEMPLATE_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/modules/data_manager_modules/data_manager_nonvolatile.h"

#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_data_messages.h"
#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"

#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "KraftKontrol/utils/buffer.h"

#include "KraftKontrol/hal/bus_device_hal_abstract.h"



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
     * @param selector For I2C this is the address. For SPI this is the chip select pin.
     * @param eeprom Reference to EEPROM module to use for calibration values.
     */
    QMC5883Driver(BusDevice_HAL_Abstract& bus, uint32_t selector, DataManager_NonVolatile* eeprom = nullptr) : Task_Abstract("QMC5883 Driver", 250, eTaskPriority_t::eTaskPriority_VeryHigh) {
        eeprom_ = eeprom;
        bus_ = &bus;
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

    Vector<> _lastMag;

    DataManager_NonVolatile* eeprom_ = nullptr;

    BusDevice_HAL_Abstract *bus_;

    uint8_t _startAttempts = 0;

    uint32_t _lastMeasurement = 0;

    Vector<> magMin_ = -1;
    Vector<> magMax_ = 1;

    bool calibrate_ = false;
    uint32_t calibrationStart_ = 0;

    eMagCalibStatus_t calibrationStatus_ = eMagCalibStatus_t::eMagCalibStatus_NotCalibrated;

    
};



#endif