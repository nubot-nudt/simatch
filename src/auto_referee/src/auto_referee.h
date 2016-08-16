#ifndef AUTO_REFEREE_H
#define AUTO_REFEREE_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "nubot/core/core.hpp"
#include "nubot_common/CoachInfo.h"
#include "nubot_common/Point2d.h"
#include "nubot/nubot_control/fieldinformation.h"
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include "Quaternion.hh"
#include <boost/bind.hpp>

#define CYAN_TEAM       -1
#define MAGENTA_TEAM    1
#define CM2M_CONVERSION 0.01
#define M2CM_CONVERSION 100
const double RAD2DEG = 180.0/M_PI;

using namespace nubot_common;
using namespace nubot;

struct ModelState
{
    std::string name;
    int         id;
    DPoint      pos;
    double      ori;
    DPoint      vel;
    double      w;

    ModelState()
    {
        name = "";
        id = -1;
        pos = DPoint(-1000.0,1000.0);
        ori = 0.0;
        vel = DPoint(0.0,0.0);
        w   = 0.0;
    }
};

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

    /// \brief auto_referee test function
    void test();

private:
    ros::NodeHandle*            rosnode_;
    ros::Publisher              cyan_pub_;
    ros::Publisher              magenta_pub_;
    ros::Subscriber             bumper_sub_;
    ros::Subscriber             gazebo_sub_;
    std::string                 cyan_prefix_;
    std::string                 magenta_prefix_;
    std::string                 ball_name_;
    CoachInfo                   cyan_coach_info_;
    CoachInfo                   magenta_coach_info_;
    gazebo_msgs::ContactsState  contacts_;
    std::vector<ModelState>     cyan_info_;
    std::vector<ModelState>     magenta_info_;
    ModelState                  ball_state_;
    FieldInformation            fieldinfo_;

};

#endif // AUTO_REFEREE_H
