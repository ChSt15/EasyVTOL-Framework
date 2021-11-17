#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



#include "KraftKontrol/data_containers/navigation_data.h"

#include "KraftKontrol/utils/data_timestamped.h"
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
        navigationData_.data.homePosition = homePosition;
        navigationData_.data.position = 0;
        navigationData_.data.velocity = 0;
    };

    /**
     * Sets the navigation data to given parameter.
     * This id to reset to a known position
     * @param startPose
     */
    virtual void setStartKinematics(const KinematicData& startPose) {
        navigationData_.data = startPose;
    }

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     *
     * @return navigation parameters.
     */
    const DataTimestamped<NavigationData>& getNavigationData() const {return navigationData_;}

    /**
     * @returns reference to nav modules output topic
     */
    const Topic<DataTimestamped<NavigationData>>& getNavigationDataTopic() const {return navigationDataTopic_;}


protected:

    //Topic to distributing new navigation data.
    Topic<DataTimestamped<NavigationData>> navigationDataTopic_;

    //Storage container for navigationData.
    DataTimestamped<NavigationData> navigationData_;


};




#endif