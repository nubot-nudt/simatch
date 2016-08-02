#include "nubot/core/core.hpp"

#include <nubot_common/VelCmd.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallInfo3d.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/TargetInfo.h>

#include <nubot_control/nubotcontrolConfig.h>
#include <dynamic_reconfigure/server.h>
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
        ballinfo3d_sub1_    = nh_->subscribe("kinect/ballinfo",1, &NuBotControl::ballInfo3dCallback, this);
        control_timer_      = nh_->createTimer(ros::Duration(0.015),&NuBotControl::loopControl,this);

        dynamic_reconfigure::Server<nubot_control::nubotcontrolConfig> reconfigureServer_;
        reconfigureServer_.setCallback(boost::bind(&NuBotControl::configure, this, _1, _2));
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
        setEthercatCommond();
    }

    void
    configure(const nubot_control::nubotcontrolConfig & config, uint32_t level)
    {/*
       kp_ = config.kp;
       kalpha_ = config.kalpha;
       kbeta_  = config.kbeta;

       m_strategy_.m_plan_.kp =  kp_;
       m_strategy_.m_plan_.kalpha =  kalpha_;
       m_strategy_.m_plan_.kbeta  =   kbeta_;*/
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
          // 多机器人策略部分........
    }




    void setEthercatCommond()
    {
        nubot_common::VelCmd command;  // 最后是解算出来的结果
        float vx,vy,w;
        bool  isturn;
        vx = m_plan_.m_behaviour_.app_vx_;
        vy = m_plan_.m_behaviour_.app_vy_;
        w  = m_plan_.m_behaviour_.app_w_;
        isturn = m_plan_.m_behaviour_.isTurn_;
        m_plan_.m_behaviour_.last_app_vx_ = m_plan_.m_behaviour_.app_vx_;
        m_plan_.m_behaviour_.last_app_vy_ = m_plan_.m_behaviour_.app_vy_;
        m_plan_.m_behaviour_.last_app_w_  = m_plan_.m_behaviour_.app_w_;
        m_plan_.m_behaviour_.app_vx_ = 0;
        m_plan_.m_behaviour_.app_vy_ = 0;
        m_plan_.m_behaviour_.app_w_  = 0;
        m_plan_.m_behaviour_.isTurn_ = false;
        command.Vx = vx ;
        command.Vy = vy ;
        command.w  = w  ;
        command.isTurn = isturn;
        motor_cmd_pub_.publish(command);
    }
    //! 中场于助攻在机器人动态传球时会出现穿过传球线的现象，在此矫正传球时候，中场与助攻的跑位点，防止传球失败
    void coorrect_target(const DPoint & start_pt, const DPoint & end_pt, const DPoint & robot_pos, DPoint & target)
    {
        std::vector < DPoint> pts;
        pts.reserve(10);
        double max_y(start_pt.y_),min_y(end_pt.y_);
        if(start_pt.y_ < end_pt.y_)
        {
            max_y = end_pt.y_;
            min_y = start_pt.y_;
        }
        DPoint upper_pt =  start_pt;
        DPoint down_pt  =  end_pt;
        if( start_pt.x_ < end_pt.x_)
        {
            upper_pt = end_pt;
            down_pt  = start_pt;
        }
        pts.push_back(down_pt);
        pts.push_back(upper_pt);
        pts.push_back(DPoint(900.0,upper_pt.y_));
        pts.push_back(DPoint(900,600));
        pts.push_back(DPoint(-900,600));
        pts.push_back(DPoint(-900.0,down_pt.y_));
        if(pnpoly(pts,robot_pos)) //表示机器人要向y600方向运动；
        {
            target.x_ =robot_pos.x_;
            if(robot_pos.x_ >upper_pt.x_ && robot_pos.x_ < upper_pt.x_ +100)
                target.x_ = upper_pt.x_+100;
            else if(robot_pos.x_ < down_pt.x_ && robot_pos.x_ > down_pt.x_ -100)
                target.x_ = down_pt.x_-100;
            else
            {
                Angle delta_ang = DPoint(upper_pt.x_ -down_pt.x_,upper_pt.y_ -down_pt.y_).angle();
                if(delta_ang.radian_ > 0)
                    target.x_ = down_pt.x_ -100;
                else
                    target.x_ = upper_pt.x_+100;
            }
            if(fabs(target.x_) > 650 *WIDTH_RATIO)
                target.x_ = 650.0*target.x_/fabs(target.x_);

            target.y_ = max_y+200;
            if(fabs(target.y_) > 500 *WIDTH_RATIO)
                target.y_ = 500.0*WIDTH_RATIO;
            if(fabs(target.y_) < 400 *WIDTH_RATIO)
                target.y_ = 400.0*WIDTH_RATIO;
        }
        else
        {
            target.x_ =robot_pos.x_;
            if(robot_pos.x_ >upper_pt.x_ && robot_pos.x_ < upper_pt.x_ +100)
                target.x_ = upper_pt.x_+100;
            else if(robot_pos.x_ < down_pt.x_ && robot_pos.x_ > down_pt.x_ -100)
                target.x_ = down_pt.x_-100;
            else
            {
                Angle delta_ang = DPoint(upper_pt.x_ -down_pt.x_,upper_pt.y_ -down_pt.y_).angle();
                if(delta_ang.radian_ > 0)
                    target.x_ = upper_pt.x_ +100;
                else
                    target.x_ = down_pt.x_-100;
            }
            if(fabs(target.x_) > 650 *WIDTH_RATIO)
                target.x_ = 650.0*target.x_/fabs(target.x_);

            target.y_ = min_y-200;
            if(fabs(target.y_) > 500 *WIDTH_RATIO)
                target.y_ = -500.0*WIDTH_RATIO;
            if(fabs(target.y_) < 400 *WIDTH_RATIO)
                target.y_ = -400.0*WIDTH_RATIO;
        }
    }

    bool pnpoly(const std::vector<DPoint> & pts, const DPoint & test_pt)
    {
        int nvert=pts.size();
        int minX(100000),maxX((-100000)),maxY(-100000),minY((100000));
        for(std::size_t i = 0; i <nvert ;i++)
        {
            if(pts[i].x_<minX)
                minX=pts[i].x_;
            if(pts[i].x_>maxX)
                maxX=pts[i].x_;
            if(pts[i].y_<minY)
                minY=pts[i].y_;
            if(pts[i].y_>maxY)
                maxY=pts[i].y_;
        }

        if (test_pt.x_ < minX || test_pt.x_ > maxX || test_pt.y_< minY || test_pt.y_ > maxY)
            return false;

        int i, j;
        bool c = false;
        for (i = 0, j = nvert-1; i < nvert; j = i++)
        {
            if ( ( (pts[i].y_>test_pt.y_) != (pts[j].y_>test_pt.y_) ) &&
                 (test_pt.x_ < (pts[j].x_-pts[i].x_) * (test_pt.y_-pts[i].y_)/double((pts[j].y_-pts[i].y_))+ pts[i].x_) )
                c = !c;
        }
        return c;
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
