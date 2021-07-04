#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/utils/topic.h"



class Navigation_Interface {
public:

    /**
     * Sets the home position.
     * All position data will be in refernce to this home position.
     *
     * @param homePosition Is the position to be used as home.
     */
    virtual void setHome(const WorldPosition& homePosition) {
        navigationData_.homePosition = homePosition;
    };

    /**
     * Sets the navigation data to given parameter.
     * This id to reset to a known position
     * @param startPose
     */
    virtual void setStartKinematics(const KinematicData& startPose) {
        navigationData_ = startPose;
    }

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     *
     * @return navigation parameters.
     */
    virtual NavigationData& getNavigationData() {return navigationData_;}

    /**
     * @returns reference to nav modules output topic
     */
    Topic<NavigationData>& getNavigationDataTopic() {return navigationDataTopic_;}


protected:

    //Topic to distributing new navigation data.
    Topic<NavigationData> navigationDataTopic_;

    //Storage container for navigationData.
    NavigationData navigationData_;


};




#endif