#ifndef ROBOT2COACH_H
#define ROBOT2COACH_H

#include "nubot/core/core.hpp"
#include <QDebug>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/CoachInfo.h>
#include <nubot/world_model/world_model.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <QtCore>

using namespace std;
namespace nubot {

class Robot2coach_info
{
public:
    vector<Robot>      RobotInfo_;       //机器人的信息
    vector<BallObject> BallInfo_;        //球的信息
    vector<vector<DPoint>>     Obstacles_;       //单个机器人识别障碍物信息
    vector<DPoint>     Opponents_;       //多个机器人障碍物融合信息

    Robot2coach_info()
    {
        RobotInfo_.resize(OUR_TEAM);
        BallInfo_.resize(OUR_TEAM);
        Obstacles_ = std::vector <std::vector<DPoint> > (OUR_TEAM,std::vector<DPoint>(MAX_OBSNUMBER_CONST));
        Opponents_.reserve(MAX_OBSNUMBER_CONST *2);
    }
};

class Robot2coach:public QThread
{
public:
    ros::Subscriber  robot2coachinfo_sub_;
    ros::Publisher   coach2robotinfo_pub_;
    ros::Timer       coachinfo_publish_timer_;
    nubot_common::CoachInfo coachinfo_publish_info_;
    Robot2coach_info robot2coach_info;
    MessageFromCoach coach2robot_info;

public:
    Robot2coach(char *argv[]);
    void run();
    void update_info(const nubot_common::WorldModelInfo & _world_msg);
    void publish(const ros::TimerEvent &);
    void operator ()();
    ~Robot2coach();
};

}

#endif // ROBOT2COACH_H
