#ifndef AUTO_REFEREE_H
#define AUTO_REFEREE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>         // Custom Callback Queue
#include <ros/subscribe_options.h>
#include "nubot/core/core.hpp"
#include "nubot_common/CoachInfo.h"
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
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

    /// \brief Custom message callback queue thread
    void message_queue_thread_fun();

    /// \brief determine which team collides with the football
    /// \return CYAN_TEAM or MAGENTA_TEAM
    int whichCollidesBall();

private:
    ros::NodeHandle*            rosnode_;
    ros::Publisher              cyan_pub_;
    ros::Publisher              magenta_pub_;
    ros::Subscriber             bumper_sub_;
    std::string                 cyan_prefix_;
    std::string                 magenta_prefix_;
    nubot_common::CoachInfo     cyan_coach_info_;
    nubot_common::CoachInfo     magenta_coach_info_;
    ros::CallbackQueue          message_queue_;     // Custom Callback Queue. Details see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    boost::thread               message_callback_queue_thread_;     // Thead object for the running callback Thread.
    boost::mutex                msgCB_lock_;        // A mutex to lock access to fields that are used in ROS message callbacks
    gazebo_msgs::ContactsState  contacts_;


};

#endif // AUTO_REFEREE_H
