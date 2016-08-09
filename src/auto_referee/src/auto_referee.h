#ifndef AUTO_REFEREE_H
#define AUTO_REFEREE_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "nubot/core/core.hpp"
#include "nubot_common/CoachInfo.h"
//#include "nubot/nubot_control/fieldinformation.h"
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#define CYAN_TEAM       -1
#define MAGENTA_TEAM    1

class auto_referee
{
public:
    auto_referee();
    ~auto_referee();

    /// \brief send game commands to two teams of robots
    void sendGameCommand(int id);

    /// \brief contact sensor callback function
    void contactCallback(const gazebo_msgs::ContactsState::ConstPtr& contacts);

    /// \brief gazebo model states callback function
    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& states);

    /// \brief determine which team collides with the football
    /// \return CYAN_TEAM or MAGENTA_TEAM
    int whichCollidesBall();

private:
    ros::NodeHandle*            rosnode_;
    ros::Publisher              cyan_pub_;
    ros::Publisher              magenta_pub_;
    ros::Subscriber             bumper_sub_;
    ros::Subscriber             gazebo_sub_;
    std::string                 cyan_prefix_;
    std::string                 magenta_prefix_;
    std::string                 ball_name_;
    nubot_common::CoachInfo     cyan_coach_info_;
    nubot_common::CoachInfo     magenta_coach_info_;
    gazebo_msgs::ContactsState  contacts_;
    gazebo_msgs::ModelStates    cyan_info_;
    gazebo_msgs::ModelStates    magenta_info_;
    gazebo_msgs::ModelState     ball_state_;
    //nubot::FieldInformation     fieldinfo_;


};

#endif // AUTO_REFEREE_H
