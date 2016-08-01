#include <iostream>
#include <fstream>
#include <sstream>
#include "nubot/core/core.hpp"
#include "nubot/world_model/world_model.h"
#include <ros/ros.h>
#include <boost/thread.hpp>
using namespace nubot;
int main(int argc, char **argv)
{
    ROS_INFO("start world_model process");
    ros::Time::init();
    World_Model this_world_model(argc,argv,"world_model_node");
    ros::spin();
    return 0;
}
