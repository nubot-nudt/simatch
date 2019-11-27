#include "core.hpp"

#include <nubot_common/VelCmd.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallInfo3d.h>
//#include <nubot_common/BallHandle.h>
//#include <nubot_common/Shoot.h>
#include <nubot_common/ActionCmd.h>
#include <nubot_common/BallIsHolding.h>
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
    ros::Subscriber  ballisholding_sub_;
//    ros::ServiceClient ballhandle_client_;
//    ros::ServiceClient shoot_client_;

    ros::Publisher   motor_cmd_pub_;
    ros::Publisher   strategy_info_pub_;
    ros::Publisher   action_cmd_pub_;
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
    Angle   robot_ori_;
    double  robot_w;
    DPoint  ball_pos_;
    DPoint  ball_vel_;
    bool    isactive    =false; ///active role
    bool    shoot_flag  =false;
    int     shoot_count = 0;
    nubot_common::BallIsHolding ball_holding_;
    nubot_common::ActionCmd     action_cmd_;
    nubot_common::VelCmd        vel;


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
    ROS_FATAL("control_robot_name:%s",robot_name.c_str());
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
        action_cmd_pub_ = nh_->advertise<nubot_common::ActionCmd>("nubotcontrol/actioncmd",1);

        //        std::string  service = "BallHandle";
//        ballhandle_client_ =  nh_->serviceClient<nubot_common::BallHandle>(service);
//        std::string  service1 = "Shoot";
//        shoot_client_ = nh_->serviceClient<nubot_common::Shoot>(service1);
        worldmodelinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &NuBotControl::update_world_model_info,this);
        ballisholding_sub_ = nh_->subscribe("ballisholding/BallIsHolding",1,&NuBotControl::update_ballisholding,this);
        //ballinfo3d_sub1_    = nh_->subscribe("kinect/ballinfo",1, &NuBotControl::ballInfo3dCallback, this);
        control_timer_      = nh_->createTimer(ros::Duration(0.015),&NuBotControl::loopControl,this);
        world_model_info_.AgentID_ = atoi(environment); /** 机器人标号*/
        world_model_info_.CoachInfo_.MatchMode = STOPROBOT;
        m_plan_.world_model_ =  & world_model_info_;
        m_plan_.m_subtargets_.world_model_ =  & world_model_info_;
        m_staticpass_.world_model_= & world_model_info_;
        m_strategy_ = new Strategy(world_model_info_,m_plan_);
        ball_holding_.BallIsHolding = 0;
        action_cmd_.maxvel=MAXVEL;
        action_cmd_.maxw=MAXW;
        action_cmd_.move_action=No_Action;
        action_cmd_.rotate_acton=No_Action;
        action_cmd_.rotate_mode=1;
        action_cmd_.shootPos = 0;
        action_cmd_.strength = 0;
        action_cmd_.handle_enable = 0;
    }

    ~NuBotControl()
    {
        m_plan_.m_behaviour_.app_vx_ = 0;
        m_plan_.m_behaviour_.app_vy_ = 0;
        m_plan_.m_behaviour_.app_w_  = 0;
        ball_holding_.BallIsHolding = 0;
        action_cmd_.shootPos = 0;
        action_cmd_.strength = 0;
        action_cmd_.handle_enable = 0;
        action_cmd_pub_.publish(action_cmd_);
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
//            if(world_model_info_.AgentID_ != i+1)
//            {
                world_model_info_.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
                world_model_info_.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
                world_model_info_.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
                world_model_info_.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
//            }
        }
        /** 更新障碍物信息*/
        world_model_info_.Obstacles_.clear();
        for(nubot_common::Point2d point : _world_msg.obstacleinfo.pos )
            world_model_info_.Obstacles_.push_back(DPoint(point.x,point.y));
//        std::cout<<"obstacles "<<world_model_info_.Obstacles_.size()<<"  "<<world_model_info_.AgentID_<<std::endl;
        world_model_info_.Opponents_.clear();
        for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
            world_model_info_.Opponents_.push_back(DPoint(point.x,point.y));
///        std::cout<<"opponents "<<world_model_info_.Opponents_.size()<<"  "<<world_model_info_.AgentID_<<std::endl;
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

        /** 这个先如此改，之后将所有数据用world_model_进行传递*/
        m_strategy_->goalie_strategy_.robot_info_    = _world_msg.robotinfo[world_model_info_.AgentID_-1];
        m_strategy_->goalie_strategy_.ball_info_2d_  = _world_msg.ballinfo[world_model_info_.AgentID_-1];
    }

    void
    update_ballisholding(const nubot_common::BallIsHolding & ball_holding)
    {
        ball_holding_.BallIsHolding=ball_holding.BallIsHolding;
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

        if(match_mode_ == STOPROBOT )
        {
            /// 运动参数
            action_cmd_.move_action =No_Action;
            action_cmd_.rotate_acton=No_Action;
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
        handleball();
        setEthercatCommand();
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
        DPoint target;
        DPoint br = ball_pos_ - robot_pos_;
        switch(world_model_info_.AgentID_)  // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
        {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
        case 1:
            target = DPoint(-1050.0,0.0);
            break;
        case 2:
            target = DPoint(-200.0,100.0);
            break;
        case 3:
            target = DPoint(-200.0,-100.0);
            break;
        case 4:
            target = DPoint(-550.0,200.0);
            break;
        case 5:
            target = DPoint(-550.0,-200.0);
            break;
        }
        if(target.distance(ball_pos_)<300&&!world_model_info_.field_info_.isOurPenalty(target))
            target = ball_pos_.pointofline(target,320.0);
        if(move2target(target, robot_pos_))
            move2ori(br.angle().radian_, robot_ori_.radian_);
        action_cmd_.move_action = Positioned_Static;
        action_cmd_.rotate_acton= Positioned_Static;
        action_cmd_.rotate_mode = 0;
    }
    void  OurDefaultReady_()
    {
        DPoint br = ball_pos_ - robot_pos_;
        DPoint target;
        switch(world_model_info_.AgentID_)  // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
        {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
        case 1:
            target = DPoint(-1050.0,0.0);
            break;
        case 2:
            target = ball_pos_.pointofline(robot_pos_,100.0);
            break;
        case 3:
            target = ball_pos_.pointofline(robot_pos_,200.0);
            break;
        case 4:
            target = DPoint(-550.0,200.0);
            break;
        case 5:
            target = DPoint(-550.0,-200.0);
            break;
        }
        if(move2target(target, robot_pos_))
            move2ori(br.angle().radian_, robot_ori_.radian_);
        action_cmd_.move_action = Positioned_Static;
        action_cmd_.rotate_acton= Positioned_Static;
        action_cmd_.rotate_mode = 0;
    }

    void parking()
    {
        static double parking_y=-680.0;
        cout<<"PARKINGROBOT"<<endl;
        DPoint parking_target;
        float tar_ori = SINGLEPI_CONSTANT/2.0;
        parking_target.x_= FIELD_XLINE7 + 150.0 * world_model_info_.AgentID_;
//        if(world_model_info_.AgentID_ == 1)
//            parking_target.x_ = -900;//守门员站在离球门最近的地方
        parking_target.y_ = parking_y;

        if(move2target(parking_target, robot_pos_))    //停到目标点10cm附近就不用动了，只需调整朝向
            move2ori(tar_ori, robot_ori_.radian_);
        action_cmd_.move_action = Positioned_Static;
        action_cmd_.rotate_acton= Positioned_Static;
        action_cmd_.rotate_mode = 0;
    }

    void handleball()
    {
        if(isactive&&match_mode_==STARTROBOT&&!shoot_flag)
            action_cmd_.handle_enable = 1;
        else
            action_cmd_.handle_enable = 0;
    }

    void normalGame()
    {
        static bool last_dribble = 0;
        isactive =false;
        if(world_model_info_.AgentID_ != 1 && isNearestRobot())
        {
            isactive=true;
        }
        if(isactive && !shoot_flag)
        {
            DPoint b2r = ball_pos_ - robot_pos_;
            DPoint tmp(200.0,300.0);
            DPoint t2r = tmp - robot_pos_;
            DPoint shoot_line = world_model_info_.field_info_.oppGoal_[GOAL_MIDDLE] - robot_pos_;
            if(last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getDribbleState())
                ROS_INFO("change::");
            last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getDribbleState();

            if(!world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getDribbleState())
            {
                action_cmd_.handle_enable = 1;
                if(move2ori(b2r.angle().radian_,robot_ori_.radian_))
                    move2target(ball_pos_,robot_pos_);
                action_cmd_.move_action = CatchBall;
                action_cmd_.rotate_acton= CatchBall;
                action_cmd_.rotate_mode = 0;
            }
            else if(robot_pos_.distance(tmp)>30.0)
            {
                action_cmd_.move_action = MoveWithBall;
                action_cmd_.rotate_acton= MoveWithBall;
                action_cmd_.rotate_mode = 0;
                if(move2ori(t2r.angle().radian_,robot_ori_.radian_))
                    move2target(tmp,robot_pos_);
            }
            else
            {
                action_cmd_.move_action = TurnForShoot;
                action_cmd_.rotate_acton= TurnForShoot;
                action_cmd_.rotate_mode = 0;
                move2target(tmp,robot_pos_);
                move2ori(shoot_line.angle().radian_,robot_ori_.radian_);
                {
                    double up_radian_  = (world_model_info_.field_info_.oppGoal_[GOAL_MIDUPPER] - robot_pos_).angle().radian_;
                    double low_radian_ = (world_model_info_.field_info_.oppGoal_[GOAL_MIDLOWER] - robot_pos_).angle().radian_;
                    if(robot_ori_.radian_>low_radian_ && robot_ori_.radian_<up_radian_)
                    {
                        action_cmd_.shootPos = RUN/*FLY*/;
                        action_cmd_.strength = shoot_line.length()/100;
                        if(action_cmd_.strength<3.0)
                            action_cmd_.strength = 3.0;
                        shoot_flag = true;
                        std::cout<<"shoot done "<<std::endl;
                    }
                }
            }
        }
        else
        {
            action_cmd_.move_action=No_Action;
            action_cmd_.rotate_acton=No_Action;
            if(shoot_flag)
                shoot_count++;
            if(shoot_count>20)
            {
                shoot_count=0;
                shoot_flag=false;
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

    bool move2target(DPoint target, DPoint pos, double distance_thres=20.0)     // 一个十分简单的实现，可以用PID
    {
        action_cmd_.target.x = target.x_;
        action_cmd_.target.y = target.y_;
        action_cmd_.maxvel = pos.distance(target);
        if(pos.distance(target) > distance_thres)
            return false;
        else
            return true;
    }

    bool move2ori(double target, double angle, double angle_thres = 8.0*DEG2RAD)  // 一个十分简单的实现，可以用PID
    {
        action_cmd_.target_ori =target;
        action_cmd_.maxw = fabs(target-angle)*2;
        if(fabs(target-angle) > angle_thres)        // 容许误差为5度
            return false;
        else
            return true;
    }

    void setEthercatCommand()
    {
        /// initialize the command
        nubot_common::ActionCmd command;
        command.move_action  =No_Action;
        command.rotate_acton =No_Action;
        command.rotate_mode  =0;
        command.maxvel = 0;
        command.maxw   = 0;
        command.target_w   =0;
        /// 机器人人位置信息 robot states
        command.robot_pos.x=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation().x_;
        command.robot_pos.y=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation().y_;
        command.robot_vel.x=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getVelocity().x_;
        command.robot_vel.y=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getVelocity().y_;
        command.robot_ori=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getHead().radian_;
        command.robot_w=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getW();
        /// 运动参数
        command.move_action =action_cmd_.move_action;
        command.rotate_acton=action_cmd_.rotate_acton;
        command.rotate_mode =action_cmd_.rotate_mode;
        command.target      =action_cmd_.target;
        command.target_vel  =action_cmd_.target_vel;
        command.target_w    =action_cmd_.target_w;
        command.target_ori  =action_cmd_.target_ori;
        command.maxvel      =action_cmd_.maxvel;
        command.maxw        =action_cmd_.maxw;
        if(command.maxvel>MAXVEL)
            command.maxvel=MAXVEL;
        if(command.maxw>MAXW)
            command.maxw=MAXW;
        if(fabs(command.target_ori)>10000.0)
            command.target_ori = 0;
        /// 带球及射门选择
        command.handle_enable=action_cmd_.handle_enable;
        command.strength=action_cmd_.strength;
        if(command.strength!= 0)
            std::cout<<"passed out"<<command.strength<<std::endl;
        command.shootPos = action_cmd_.shootPos;
        /// 传一次后，力量清0,防止多次射门
        action_cmd_.strength=0;
        action_cmd_pub_.publish(command);
    }
    void pubStrategyInfo()
    {
        nubot_common::StrategyInfo strategy_info;       // 这个消息的定义可以根据个人需要进行修改
        strategy_info.header.stamp = ros::Time::now();
        strategy_info.AgentID     = world_model_info_.AgentID_;
        strategy_info.is_dribble = ball_holding_.BallIsHolding;
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
