#ifndef _NUBOT_World_Model_H_
#define _NUBOT_World_Model_H_

#include "world_model/teammatesinfo.h"
#include "Fieldinformation.h"
#include "rtdb/rtdb_api.h"
#include "rtdb/rtdb_user.h"

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <nubot_common/BallInfo.h>
#include <nubot_common/RobotInfo.h>
#include <nubot_common/ObstaclesInfo.h>
#include <nubot_common/OminiVisionInfo.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/TargetInfo.h>
#include <nubot_common/BallInfo3d.h>
#include <nubot_common/object_info.h>
#include <semaphore.h>

#ifdef SIMULATION
#include "rtdb/multicast.h"
#include "nubot_common/simulation_strategy.h"
#endif

namespace nubot {
class World_Model
{

public:
    World_Model(int argc,char** argv,const char * name);
    ~World_Model();
    void update(const ros::TimerEvent& event);
    void updateOminivision(const nubot_common::OminiVisionInfo &omniinfo);
    void updateKinectBall(const nubot_common::object_info  &tx1_info);
    void updateInfo();
    void publish();
    void sendToTeamnates();
    bool IsLocationInField(DPoint location);
public:
    std::vector<Teammatesinfo> teammatesinfo_;
    Ball ball_info_;
    Obstacles obstacles_;
    FieldInformation field_info_;

    int  teammateIDforBallSelected ;
    int  AgentID_;
    int  coach_socket_;

    ros::Time omni_update_time_;    /** 全向视觉节点发布的topic更新时间；*/
    ros::Time kinect_update_time_;  /** kinect节点发布的topic更新时间；*/
    ros::Time front_update_time_;   /** 前向视觉节点发布的topic更新时间；*/
    ros::Time nubot_control_time_;
    MessageFromCoach coach2robot_;

public:
    nubot_common::WorldModelInfo world_model_info_;
    ros::Publisher    worldmodelinfo_pub_;
    ros::Subscriber   omin_vision_sub_;
    ros::Subscriber   strategy_info_sub_;
    ros::Subscriber   target_info_sub_;
    ros::Subscriber   front_vision_sub_;
    ros::Subscriber   kinect_vision_sub_; 
    ros::Timer        worldmodel_update_timer_;
    ros::Time         receive_coach_count_;
    boost::shared_ptr<ros::NodeHandle> nh;

#ifdef SIMULATION
    ros::Subscriber   coach_sub_;
    void receiveFromCoach(const nubot_common::CoachInfo & _coach);
    void updateStrategyinfo(const nubot_common::simulation_strategy & _strategyinfo);
#else
    void updateStrategyinfo(const nubot_common::StrategyInfo &strategyinfo);
#endif
};

}


#endif // _NUBOT_World_Model_H_
