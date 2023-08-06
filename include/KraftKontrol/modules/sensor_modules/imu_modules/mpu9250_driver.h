#ifndef MPU9250_DRIVER_H
#define MPU9250_DRIVER_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"

#include "KraftKontrol/platforms/arduino_platform/arduino_pin_interrupt_hal.h"

#include "KraftKontrol/modules/data_manager_modules/data_manager_nonvolatile.h"

#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/MPU9250_Lib/src/mpu9250.h"

#include "KraftKontrol/utils/buffer.h"
#include "lib/MathHelperLibrary/FML.h"




class MPU9250Driver: public Gyroscope_Abstract, public Accelerometer_Abstract, public Module_Abstract, public Task_Threading {
public:

    MPU9250Driver(int interruptPin, int chipSelect, SPIClass* spiBus): Task_Threading("MPU9250 Driver", eTaskPriority_t::eTaskPriority_Realtime, SECONDS/32000), pinInterrupt_(interruptPin, _interruptRoutine, true, false), _imu(spiBus, chipSelect) {
        task_ = this;
    }

    ~MPU9250Driver() {
        task_ = nullptr;
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
     * Starts calibration sequence.
     */
    //void startCalibration() {calibrate_ = true; accelCalibStateTime_ = calibrationStart_ = NOW();}

    /**
     * Stops calibration sequence.
     */
    //void stopCalibration() {calibrate_ = false;}


private:

    static void _interruptRoutine();
    static Task_Threading* task_;

    void _getData();

    bool getEEPROMData();
    //bool setEEPROMData();

    //bool calibrate_ = false;
    //int64_t calibrationStart_ = 0;

    //Buffer<float, 100> calibBuf_;

    VectorOLD<> _lastGyro;
    VectorOLD<> _lastAccel;
    VectorOLD<> _lastMag;

    PinInterrupt_HAL pinInterrupt_;

    Mpu9250 _imu;

    uint8_t _startAttempts = 0;

    int64_t _lastMeasurement = 0;

    static int64_t _newDataTimestamp;
    static bool _newDataInterrupt;

    VectorOLD<> accelMin_ = VectorOLD<>(-9.81);
    VectorOLD<> accelMax_ = VectorOLD<>(9.81);

    //eMagCalibStatus_t calibrationStatus_ = eMagCalibStatus_t::eMagCalibStatus_NotCalibrated;

    //uint8_t accelCalibState_ = 0;
    //int64_t accelCalibStateTime_ = 0;
    
};



#endif