#ifndef MAGNETOMETER_ABSTRACT_H
#define MAGNETOMETER_ABSTRACT_H



#include "lib/MathHelperLibrary/FML.h"

#include "../../module_abstract.h"
#include "../../../utils/topic_subscribers.h"
#include "../../../KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"
#include "../../communication_modules/kraft_message_subscriber.h"

#include "../sensordata.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Magnetometer_Abstract {
private:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> valueTopic_;

    CommandMessageMagCalValues calValues_;
    CommandMessageMagMountTransform mountValues_;

    KraftMessage_Subscriber messageSubr_;


protected:

    Magnetometer_Abstract();


public:

    /**
     * @returns reference to Mag data topic
     */
    const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& getMagTopic() const;

    /**
     * Sets the magnetometer axis scaling and aligning calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisScaleAlign Matrix to scale and align magnetometer axis.
     */
    void setMagCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign);

    /**
     * @returns axisScaleAlign matrix used for calibration.
     */
    const FML::Matrix33_F& getMagCalibrationAxisScaleAlign() const;
    
    /**
     * Sets the magnetometer bias calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisBias Vector bias for each axis.
     */
    void setMagCalibrationAxisBias(const FML::Vector3_F& axisBias);

    /**
     * @returns axisBias vector used for calibration.
     */
    const FML::Vector3_F& getMagCalibrationAxisBias() const;

    /**
     * Sets the magnetometer transformation matrix.
     * Transformation formula: a' = Sa
     * With:    a' transformed value
     *          a  calibrated value
     *          S  transformation matrix
     * @param mountTransform Matrix to transform the data into local frame coordinates.
     */
    void setMagTransform(const FML::Matrix33_F& mountTransform);

    /**
     * @returns transformation matrix used to transform values into local frame.
     */
    const FML::Matrix33_F& getMagTransform() const;

    /**
     * Transforms magnetometer data and then publishes it to topic.
     * @param magValues
     * @param calibrate If true then the values will be calibrated.
     * @param transform If true then the values will be transformed to local frame.
     */
    void publishMagData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> magValues, bool calibrate = true, bool transform = true);

    
};





#endif