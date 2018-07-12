#include "nubot/core/core.hpp"

#include <nubot_common/VelCmd.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallInfo3d.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/TargetInfo.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/strategy.hpp>
#include <nubot/nubot_control/common.hpp>
#include <nubot/nubot_control/plan.h>
#include <nubot/nubot_control/staticpass.h>

#define RUN 1
#define FLY -1
const double DEG2RAD = 1.0/180.0*SINGLEPI_CONSTANT;     // 角度到弧度的转换

using namespace std;
namespace nubot{

class NuBotControl
{

public:
    ros::Subscriber  ballinfo3d_sub1_;
    ros::Subscriber  odoinfo_sub_;
    ros::Subscriber  obstaclesinfo_sub_;
    ros::Subscriber  worldmodelinfo_sub_;

    ros::ServiceClient ballhandle_client_;
    ros::ServiceClient shoot_client_;

    ros::Publisher   motor_cmd_pub_;
    ros::Publisher   strategy_info_pub_;
    ros::Timer       control_timer_;

    boost::shared_ptr<ros::NodeHandle> nh_;
public:
    World_Model_Info world_model_info_;  /** 世界模型中的信息赋值，来源于world_model节点的topic*/
    Strategy  * m_strategy_;
    Plan   m_plan_;
    StaticPass m_staticpass_;

    double kp_;
    double kalpha_;
    double kbeta_;
    char    match_mode_;
    char    pre_match_mode_;
    DPoint  robot_pos_;
    Angle  robot_ori_;
    DPoint  ball_pos_;
    DPoint  ball_vel_;


public:
    NuBotControl(int argc, char **argv)
    {
        const char * environment;
        ROS_INFO("initialize control process");

#ifdef SIMULATION
	std::string robot_name = argv[1];
	std::string num = robot_name.substr(robot_name.size()-1);
	//std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
	environment = num.c_str();
	ROS_FATAL("robot_name:%s",robot_name.c_str());
	nh_ = boost::make_shared<ros::NodeHandle>(robot_name);
#else
        nh_ = boost::make_shared<ros::NodeHandle>();
        // 读取机器人标号，并赋值. 在 .bashrc 中输入export AGENT=1，2，3，4，等等；
        if((environment = getenv("AGENT"))==NULL)
        {
            ROS_ERROR("this agent number is not read by robot");
            return ;
        }
#endif
        motor_cmd_pub_ = nh_->advertise<nubot_common::VelCmd>("nubotcontrol/velcmd",1);
        strategy_info_pub_ =  nh_->advertise<nubot_common::StrategyInfo>("nubotcontrol/strategy",10);
        std::string  service = "BallHandle";
        ballhandle_client_ =  nh_->serviceClient<nubot_common::BallHandle>(service);
        std::string  service1 = "Shoot";
        shoot_client_ = nh_->serviceClient<nubot_common::Shoot>(service1);
        worldmodelinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &NuBotControl::update_world_model_info,this);
        //ballinfo3d_sub1_    = nh_->subscribe("kinect/ballinfo",1, &NuBotControl::ballInfo3dCallback, this);
        control_timer_      = nh_->createTimer(ros::Duration(0.015),&NuBotControl::loopControl,this);

        world_model_info_.AgentID_ = atoi(environment); /** 机器人标号*/
        world_model_info_.CoachInfo_.MatchMode = STOPROBOT;
        m_plan_.world_model_ =  & world_model_info_;
        m_plan_.m_subtargets_.world_model_ =  & world_model_info_;
        m_staticpass_.world_model_= & world_model_info_;
        m_strategy_ = new Strategy(world_model_info_,m_plan_);
    }

    ~NuBotControl()
    {
        m_plan_.m_behaviour_.app_vx_ = 0;
        m_plan_.m_behaviour_.app_vy_ = 0;
        m_plan_.m_behaviour_.app_w_  = 0;
    }

    void
    update_world_model_info(const nubot_common::WorldModelInfo & _world_msg)
    {
        /** 更新PathPlan自身与队友的信息，自身的策略信息记住最好不要更新，因为本身策略是从此传过去的*/
        for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
        {
            world_model_info_.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
            world_model_info_.RobotInfo_[i].setTargetNum(1,_world_msg.robotinfo[i].targetNum1);
            world_model_info_.RobotInfo_[i].setTargetNum(2,_world_msg.robotinfo[i].targetNum2);
            world_model_info_.RobotInfo_[i].setTargetNum(3,_world_msg.robotinfo[i].targetNum3);
            world_model_info_.RobotInfo_[i].setTargetNum(4,_world_msg.robotinfo[i].targetNum4);
            world_model_info_.RobotInfo_[i].setpassNum(_world_msg.robotinfo[i].staticpassNum);
            world_model_info_.RobotInfo_[i].setcatchNum(_world_msg.robotinfo[i].staticcatchNum);

            world_model_info_.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,
                                                               _world_msg.robotinfo[i].pos.y));
            world_model_info_.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
            world_model_info_.RobotInfo_[i].setVelocity(DPoint( _world_msg.robotinfo[i].vtrans.x,
                                                                _world_msg.robotinfo[i].vtrans.y));
            world_model_info_.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
            world_model_info_.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
            world_model_info_.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
            world_model_info_.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
            /** 信息是来源于队友，则要更新机器人策略信息*/
            if(world_model_info_.AgentID_ != i+1)
            {
                world_model_info_.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
                world_model_info_.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
                world_model_info_.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
                world_model_info_.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
            }
        }
        /** 更新障碍物信息*/
        world_model_info_.Obstacles_.clear();
        for(nubot_common::Point2d point : _world_msg.obstacleinfo.pos )
            world_model_info_.Obstacles_.push_back(DPoint(point.x,point.y));
            world_model_info_.Opponents_.clear();
        for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
            world_model_info_.Opponents_.push_back(DPoint(point.x,point.y));
        /** 更新足球物信息*/
        for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
        {
            world_model_info_.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
            world_model_info_.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),
                                                                  _world_msg.ballinfo[i].real_pos.radius));
            world_model_info_.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
            world_model_info_.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
            world_model_info_.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
            world_model_info_.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
        }
        world_model_info_.BallInfoState_ = _world_msg.ballinfo[world_model_info_.AgentID_-1].ballinfostate;

        /** 更新的COACH信息*/
        world_model_info_.CoachInfo_.MatchMode =_world_msg.coachinfo.MatchMode;
        world_model_info_.CoachInfo_.MatchType =_world_msg.coachinfo.MatchType;

        /** 更新传球信息*/
        world_model_info_.pass_cmds_.catchrobot_id  = _world_msg.pass_cmd.catch_id;
        world_model_info_.pass_cmds_.passrobot_id   = _world_msg.pass_cmd.pass_id;
        world_model_info_.pass_cmds_.isvalid        = _world_msg.pass_cmd.is_valid;
        world_model_info_.pass_cmds_.is_dynamic_pass   = _world_msg.pass_cmd.is_dynamic_pass;
        world_model_info_.pass_cmds_.is_static_pass    = _world_msg.pass_cmd.is_static_pass;
        world_model_info_.pass_cmds_.is_passout = _world_msg.pass_cmd.is_passout;
        world_model_info_.pass_cmds_.pass_pt    = DPoint(_world_msg.pass_cmd.pass_pt.x,_world_msg.pass_cmd.pass_pt.y);
        world_model_info_.pass_cmds_.catch_pt   = DPoint(_world_msg.pass_cmd.catch_pt.x,_world_msg.pass_cmd.catch_pt.y);

        /*  if(world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].isStuck())
            ROS_INFO("STUCK:(YES)");
        else
            ROS_INFO("STUCK:(NO)");*/
        /** 这个先如此改，之后将所有数据用world_model_进行传递*/
        m_strategy_->goalie_strategy_.robot_info_    = _world_msg.robotinfo[world_model_info_.AgentID_-1];
        m_strategy_->goalie_strategy_.ball_info_2d_  = _world_msg.ballinfo[world_model_info_.AgentID_-1];
    }

    /** 球的三维信息,用于守门员角色*/
    void
    ballInfo3dCallback(const nubot_common::BallInfo3d  &_BallInfo_3d){

        //m_strategy_->goalie_strategy_.setBallInfo3dRel( _BallInfo_3d );
    }
    /** 主要的控制框架位于这里*/
    void
    loopControl(const ros::TimerEvent& event)
    {
        match_mode_ = world_model_info_.CoachInfo_.MatchMode;               //! 当前比赛模式
        pre_match_mode_ = world_model_info_.CoachInfo_.MatchType;           //! 上一个比赛模式
        robot_pos_  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation();
        robot_ori_  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getHead();
        ball_pos_   = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getGlobalLocation();
        ball_vel_   = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getVelocity();
        nubot_common::VelCmd        vel;
        nubot_common::Shoot         shoot;
        nubot_common::BallHandle    dribble;

        if(match_mode_ == STOPROBOT )
        {
            vel.Vx = 0;
            vel.Vy = 0;
            vel.w  = 0;
            motor_cmd_pub_.publish(vel);
            shoot.request.strength = 0;
            shoot_client_.call(shoot);
            dribble.request.enable = 0;
            ballhandle_client_.call(dribble);
        }
        /** 机器人在开始之前的跑位. 开始静态传接球的目标点计算*/
        else if(match_mode_ > STOPROBOT && match_mode_ <= DROPBALL)
            positioning();
        else if(match_mode_==PARKINGROBOT)
            parking();
        else// 机器人正式比赛了，进入start之后的机器人状态
        {
            normalGame();
        } // start部分结束

        pubStrategyInfo();  // 发送策略消息让其他机器人看到，这一部分一般用于多机器人之间的协同
    }


    void positioning()
    { 
        switch (match_mode_)
        {
        case OUR_KICKOFF:
            //OurkickoffReady_();
            OurDefaultReady_();
            break;
        case OPP_KICKOFF:
            //OppkickoffReady_();
            OppDefaultReady_();
            break;
        case OUR_FREEKICK:
            OurDefaultReady_();
            break;
        case OPP_FREEKICK:
            //OppDefaultReady_();
            OppDefaultReady_();
            break;
        case OUR_GOALKICK:
            OurDefaultReady_();
            break;
        case OPP_GOALKICK:
            //OppDefaultReady_();
            OppDefaultReady_();
            break;
        case OUR_CORNERKICK:
            OurDefaultReady_();
            break;
        case OPP_CORNERKICK:
            //OppDefaultReady_();
            OppDefaultReady_();
            break;
        case OUR_THROWIN:
            OurDefaultReady_();
            break;
        case OPP_THROWIN:
            //OppDefaultReady_();
            OppDefaultReady_();
            break;
        case OUR_PENALTY:
            //OurPenaltyReady_();
            OurDefaultReady_();
            break;
        case OPP_PENALTY:
            //OppPenaltyReady_();
            OppDefaultReady_();
            break;
        case DROPBALL:
            //DropBallReady_();
            OurDefaultReady_();
            break;
        default:
            break;
        }
    }


    void  OppDefaultReady_()
    {
         DPoint br = ball_pos_ - robot_pos_;
        switch(world_model_info_.AgentID_)  // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
        {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
            case 1:
                if(move2target(DPoint(-850.0, 0.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 2:
                if(move2target(DPoint(-150.0, 100.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 3:
                if(move2target(DPoint(-150.0, -100.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 4:
                if(move2target(DPoint(-450.0, 200.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 5:
                if(move2target(DPoint(-450.0, -200.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
        }

    }
    void  OurDefaultReady_()
    {
        DPoint br = ball_pos_ - robot_pos_;
        switch(world_model_info_.AgentID_)  // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
        {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
            case 1:
                if(move2target(DPoint(-850.0, 0.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 2:
                if(move2target(ball_pos_, robot_pos_,100.0))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 3:
                if(move2target(ball_pos_, robot_pos_,200.0))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 4:
                if(move2target(DPoint(-450.0, 200.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
            case 5:
                if(move2target(DPoint(-450.0, -200.0), robot_pos_))
                    move2ori(br.angle().radian_, robot_ori_.radian_);
            break;
        }

    }

    void parking()
    {
        static double parking_y=-580.0;
        cout<<"PARKINGROBOT"<<endl;

        DPoint parking_target;
        float tar_ori = SINGLEPI_CONSTANT/2.0;
        parking_target.x_= -120.0 * world_model_info_.AgentID_;
        if(world_model_info_.AgentID_ == 1)
            parking_target.x_ = -700;//守门员站在离球门最近的地方
        parking_target.y_ = parking_y;

        if(move2target(parking_target, robot_pos_))    //停到目标点10cm附近就不用动了，只需调整朝向
            move2ori(tar_ori, robot_ori_.radian_);
    }

    void normalGame()
    {
        if(world_model_info_.AgentID_ != 1 && isNearestRobot())
        {
            nubot_common::BallHandle    dribble;
            DPoint br = ball_pos_ - robot_pos_;

            if(move2ori(br.angle().radian_, robot_ori_.radian_))        // 先往足球靠近
            {
                if(move2target(ball_pos_, robot_pos_, 50.0))
                {
                    dribble.request.enable = 1;
                    ballhandle_client_.call(dribble);
                    if(dribble.response.BallIsHolding != true)
                    {
                        if(move2target(ball_pos_, robot_pos_, 40.0))
                            move2ori(br.angle().radian_, robot_ori_.radian_);
                    }
                    else        // 带上球了
                    {
                        DPoint tmp(200.0,300.0);
                        DPoint dirc = DPoint(900.0 ,0.0) - tmp;         // 对准 (900.0 ,0.0)
                        if(move2target(tmp, robot_pos_) &&
                           move2ori(dirc.angle().radian_, robot_ori_.radian_, 5.0*DEG2RAD))  // 跑到位以及转到位
                        {
                            nubot_common::Shoot shoot;
                            shoot.request.ShootPos = FLY;
                            shoot.request.strength = 1.0;   // 在FLY模式下，strength不重要,只要非零就行
                            shoot_client_.call(shoot);
                        }
                    }
                }
            }
        }
    }

    bool isNearestRobot()         //找到距离足球最近的机器人
    {
        float distance_min = 2000.0;
        float distance = distance_min;
        int robot_id = -1;

        for(int i=1;i<OUR_TEAM;i++)     // 排除守门员
            if(world_model_info_.RobotInfo_[i].isValid())
            {
                distance = ball_pos_.distance(world_model_info_.RobotInfo_[i].getLocation());
                if(distance < distance_min)
                {
                    distance_min=distance;
                    robot_id = i;
                }
            }
        if(robot_id+1 == world_model_info_.AgentID_)
            return true;
        else
            return false;
    }

    bool move2target(DPoint target, DPoint pos, double distance_thres=10.0)     // 一个十分简单的实现，可以用PID
    {
        static nubot_common::VelCmd        vel;
        DPoint tmp = target - pos;
        float tar_theta = tmp.angle().radian_;

        if(tmp.norm() > distance_thres)
        {
            vel.Vx = tmp.norm()*0.7*cos(tar_theta - robot_ori_.radian_);    // 注意将全局坐标系下的期望速度转换为在机器人体坐标系下
            vel.Vy = tmp.norm()*0.7*sin(tar_theta - robot_ori_.radian_);
            vel.w = 0;
            motor_cmd_pub_.publish(vel);
            return false;
        }
        else
        {
            vel.Vx = 0.0;
            vel.Vy = 0.0;
            vel.w = 0;
            motor_cmd_pub_.publish(vel);
            return true;
        }
    }

    bool move2ori(double target, double angle, double angle_thres = 8.0*DEG2RAD)  // 一个十分简单的实现，可以用PID
    {
        static nubot_common::VelCmd        vel;
        double tmp = target - angle;
        if(fabs(tmp) > angle_thres)        // 容许误差为5度
        {
            vel.Vx = 0;
            vel.Vy = 0;
            vel.w = tmp;
            motor_cmd_pub_.publish(vel);
            return false;
        }
        else
        {
            vel.Vx = 0.0;
            vel.Vy = 0.0;
            vel.w = 0;
            motor_cmd_pub_.publish(vel);
            return true;
        }
    }

    void pubStrategyInfo()
    {
        nubot_common::StrategyInfo strategy_info;       // 这个消息的定义可以根据个人需要进行修改
        strategy_info.header.stamp = ros::Time::now();
        strategy_info.AgentID     = world_model_info_.AgentID_;

        strategy_info_pub_.publish(strategy_info);
    }
};

}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"nubot_control_node");
    // 完成一系列的初始化工作？ 以及相应的报错机制。  只有当所有的传感器信息都已经准备就绪的时候才可以运行
    ros::Time::init();
    ROS_INFO("start control process");
    nubot::NuBotControl nubotcontrol(argc,argv);
    ros::spin();
    return 0;
}
