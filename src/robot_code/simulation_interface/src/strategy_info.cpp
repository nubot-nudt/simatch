#include "ros/ros.h"
#include "core.hpp"
#include "nubot_common/simulation_strategy.h"

#ifdef SIMULATION
nubot_common::StrategyInfo strategy_info[OUR_TEAM];
void update_bot2(const nubot_common::StrategyInfo & _strategy)
{
  strategy_info[1] =_strategy;
}
void update_bot3(const nubot_common::StrategyInfo & _strategy)
{
   strategy_info[2] =_strategy;
}
void update_bot4(const nubot_common::StrategyInfo & _strategy)
{
  strategy_info[3] =_strategy;
}
void update_bot5(const nubot_common::StrategyInfo & _strategy)
{
   strategy_info[4] =_strategy;
}
#endif

int main(int argc, char **argv)
{
#ifdef SIMULATION
    ros::init(argc, argv,"strategy_pub_node");
    ros::Time::init();
    ros::NodeHandle nh;
    std::string robot_prefix = argv[1];
    std::string topic_name = "/" + robot_prefix + "/nubotcontrol/strategy";
    ROS_FATAL("strategy_info: robot_prefix:%s",robot_prefix.c_str());

    ros::Subscriber control2 = nh.subscribe("/" + robot_prefix + "2" + "/nubotcontrol/strategy",10,update_bot2);
    ros::Subscriber control3 = nh.subscribe("/" + robot_prefix + "3" + "/nubotcontrol/strategy",10,update_bot3);
    ros::Subscriber control4 = nh.subscribe("/" + robot_prefix + "4" + "/nubotcontrol/strategy",10,update_bot4);
    ros::Subscriber control5 = nh.subscribe("/" + robot_prefix + "5" + "/nubotcontrol/strategy",10,update_bot5);
    ros::Publisher  coachinfo_pub = nh.advertise<nubot_common::simulation_strategy>(topic_name, 1);
    ros::Rate loop_rate(30);
    nubot_common::simulation_strategy strategy;
    while(ros::ok())
    {
        strategy.strategy_msgs.clear();
        for(int i = 1 ; i < OUR_TEAM; i++)
        {
            ros::Duration duration = ros::Time::now() - strategy_info[i].header.stamp;
            if(fabs(duration.toSec())<0.10)
                strategy.strategy_msgs.push_back(strategy_info[i]);
        }
        coachinfo_pub.publish(strategy);
        ros::spinOnce();
        loop_rate.sleep();
    }
#endif
    return 0;
}
