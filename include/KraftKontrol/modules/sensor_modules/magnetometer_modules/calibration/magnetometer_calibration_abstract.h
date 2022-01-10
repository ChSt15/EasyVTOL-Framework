#ifndef MAGNETOMETER_CALIBRATION_ABSTRACT
#define MAGNETOMETER_CALIBRATION_ABSTRACT



#include "lib/MathHelperLibrary/FML.h"



/**
 * Abstract class that serves as an interface class for magnetometer calibration methods.
 * Simply call calibrateValue while giving it a new raw magnetometer value and the module will update internal values of needed and calibrate given raw value. 
 * Use calibrationFinished to see if the calibration is good and resetValues to recalibrate.
 * Subclasses must implement newValue() and resetValues() methods and update the hardIron_, softIron_ and calibrationGood_ attributes.
 */
class MagnetometerCalibration_Abstract {
protected:

    //Magnetometer hard iron effects.
    FML::Vector3_F hardIron_; 

    //Magnetometer soft iron effects.
    FML::Matrix33_F softIron_;

    //Set to true to signal calibration good.
    bool calibrationGood_ = false;


public:

    virtual ~MagnetometerCalibration_Abstract() {}


protected:

    /**
     * Gives calibration method a new value.
     * @param rawValue Raw magnetometer value.
     */
    virtual void newValue(const FML::Vector3_F& rawValue) = 0;


public:

    /**
     * Resets module for new calibration.
     */
    virtual void resetValues() = 0;

    /**
     * Gives calibration module time to do stuff with values.
     * @returns true if calibration values are good and available.
     */ 
    bool calibrationFinished();

    /**
     * Gets the hard iron effects of the magnetometer. 
     * These should be subtracted from the magnetometer values after Soft Iron Effect calibration.
     * Should return 0 vector if no calibration values available.
     * @returns magnetometer hard iron bias vector.
     */
    const FML::Vector3_F& getHardIronEffect();

    /**
     * Sets the hard iron effects of the magnetometer. 
     * @param hardIronEffect Magnetometer hard iron effects after soft iron.
     */
    void setHardIronEffect(const FML::Vector3_F& hardIronEffect);

    /**
     * Gets the soft iron effects of the magnetometer. 
     * These should be multiplicated with the magnetometer values before hard iron effects bias calibration.
     * Should return identity (Diagonal matrix with 1s) matrix if no calibration values available. 
     * @returns magnetometer soft iron bias matrix.
     */
    const FML::Matrix33_F& getSoftIronEffect();

    /**
     * sets the soft iron effects of the magnetometer. 
     * @param softIronEffect Magnetometer soft iron effects before hard iron effects.
     */
    void setSoftIronEffect(const FML::Matrix33_F& softIronEffect);

    /**
     * Calibrates given value if calibration finished.
     * @param value Raw magnetometer value to be calibrated.
     */
    void calibrateValue(FML::Vector3_F &value);

    
};



#endif