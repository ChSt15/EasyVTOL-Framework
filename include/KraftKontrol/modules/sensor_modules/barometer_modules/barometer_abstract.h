#ifndef BAROMETER_ABSTRACT_H
#define BAROMETER_ABSTRACT_H



#include "stdint.h"


#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Barometer_Abstract {
public:

    /**
     * @returns reference to baro data topic
     */
    const Topic<DataTimestamped<float>>& getBaroTopic() const {return baroTopic_;}

    /**
     * Calculates height from current pressure and pressure at sea level for reference.
     * 
     * @param pressure Air pressure at current position.
     * @param refPressure Air pressure at sea level.
     * @returns altitude above reference pressure (sea level)
     */
    static float getHeightFromPressure(const float &pressure, const float &refPressure) {
        return ((float)-44330.77)*(pow(((float)pressure/(float)refPressure), 0.190263) - (float)1);
    }

    /**
     * Calculates height from current pressure and pressure at sea level for reference.
     * 
     * @param pressure Air pressure at current position.
     * @param height Current height.
     * @returns calclulated sealevel pressure.
     */
    static float getSealevelPressureFromHeight(const float &pressure, const float &height) {
        return pressure/pow(height/((float)-44330.77) + 1, 5.25588f);
    }


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<float>> baroTopic_;

    
};





#endif