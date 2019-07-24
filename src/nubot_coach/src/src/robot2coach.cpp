#include <robot2coach.h>

using namespace nubot;

Robot2coach::Robot2coach(char *argv[])
{
    ros::Time::init();
    boost::shared_ptr<ros::NodeHandle> nh_;
    #ifdef SIMULATION
    std::string robot_name = argv[1];
    nh_= boost::make_shared<ros::NodeHandle>(robot_name);
    robot2coachinfo_sub_ = nh_->subscribe("/"+robot_name+"2/worldmodel/worldmodelinfo", 1, &Robot2coach::update_siminfo,this);
    coach2robotinfo_pub_ = nh_->advertise<nubot_common::CoachInfo>("receive_from_coach",30);
    coachinfo_publish_timer_ = nh_->createTimer(ros::Duration(0.03),&Robot2coach::publish,this);
    #else
    nh_= boost::make_shared<ros::NodeHandle>();
    robot2coachinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &Robot2coach::update_info,this);
    coach2robotinfo_pub_ = nh_->advertise<nubot_common::CoachInfo>("nubot_coach/coachinfo",30);
    coachinfo_publish_timer_ = nh_->createTimer(ros::Duration(0.03),&Robot2coach::publish,this);
    #endif
}
Robot2coach::~Robot2coach()
{
    if(ros::isStarted())
    {
      ros::shutdown();
      ros::waitForShutdown();
    }
    wait();
}

//用于接受simulation worldmodel的信息
void Robot2coach::update_siminfo(const nubot_common::WorldModelInfo & _world_msg)
{
    //更新机器人信息

    for(int i=0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
        robot2coach_info.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,_world_msg.robotinfo[i].pos.y));
        robot2coach_info.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
        robot2coach_info.RobotInfo_[i].setVelocity(DPoint( _world_msg.robotinfo[i].vtrans.x,_world_msg.robotinfo[i].vtrans.y));
        robot2coach_info.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
        robot2coach_info.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
        robot2coach_info.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
        robot2coach_info.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
        robot2coach_info.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
        robot2coach_info.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
        robot2coach_info.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
//        robot2coach_info.RobotInfo_[i].setCurrentAction(_world_msg.robotinfo[i].current_action);
        robot2coach_info.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
    }
    //更新单个机器人障碍物信息
    for(int i = 0 ; i < OUR_TEAM ; i++)
    {

        if(robot2coach_info.RobotInfo_[i].isValid()&&_world_msg.obstacleinfo.pos.size()!=0)///
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                /// if _world_msg.obstacleinfo.pos is not empty,
                robot2coach_info.Obstacles_[i][j] = DPoint2s( _world_msg.obstacleinfo.pos[j].x, _world_msg.obstacleinfo.pos[j].y);
    }
    //更新融合后障碍物信息
    robot2coach_info.Opponents_.clear();
    for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
        robot2coach_info.Opponents_.push_back(DPoint2s(point.x,point.y));
    //更新足球物信息
    for(int i=0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
        robot2coach_info.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),_world_msg.ballinfo[i].real_pos.radius));
        robot2coach_info.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
        robot2coach_info.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        robot2coach_info.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
        robot2coach_info.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
    }
    //更新传接球信息
    robot2coach_info.Pass_id_=_world_msg.pass_cmd.pass_id;           //带球机器人即为传球机器人
    robot2coach_info.Catch_id_=_world_msg.pass_cmd.catch_id;         //传球机器人计算出的接球机器人
    robot2coach_info.isPass_valid_=_world_msg.pass_cmd.is_valid;
}

//用于接受worldmodel的信息
void Robot2coach::update_info(const nubot_common::CoachWorldModelInfo & _world_msg)
{
    //更新机器人信息
    for(int i=0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
        robot2coach_info.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,_world_msg.robotinfo[i].pos.y));
        robot2coach_info.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
        robot2coach_info.RobotInfo_[i].setVelocity(DPoint( _world_msg.robotinfo[i].vtrans.x,_world_msg.robotinfo[i].vtrans.y));
        robot2coach_info.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
        robot2coach_info.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
        robot2coach_info.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
        robot2coach_info.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
        robot2coach_info.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
        robot2coach_info.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
        robot2coach_info.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
//        robot2coach_info.RobotInfo_[i].setCurrentAction(_world_msg.robotinfo[i].current_action);
        robot2coach_info.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
    }

    //更新单个机器人障碍物信息
    for(int i = 0 ; i < OUR_TEAM ; i++)
        if(robot2coach_info.RobotInfo_[i].isValid())
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                robot2coach_info.Obstacles_[i][j]=DPoint2s( _world_msg.obstacleinfo[i].pos[j].x, _world_msg.obstacleinfo[i].pos[j].y);

    //更新融合后障碍物信息
    robot2coach_info.Opponents_.clear();
    for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
        robot2coach_info.Opponents_.push_back(DPoint2s(point.x,point.y));

    //更新足球物信息
    for(int i=0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
        robot2coach_info.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),_world_msg.ballinfo[i].real_pos.radius));
        robot2coach_info.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
        robot2coach_info.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        robot2coach_info.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
        robot2coach_info.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
    }

    //更新传接球信息
    robot2coach_info.Pass_id_=_world_msg.pass_cmd.pass_id;           //带球机器人即为传球机器人
    robot2coach_info.Catch_id_=_world_msg.pass_cmd.catch_id;         //传球机器人计算出的接球机器人
    robot2coach_info.isPass_valid_=_world_msg.pass_cmd.is_valid;
}

//用于发布coach的信息
void Robot2coach::publish(const ros::TimerEvent &)
{
    coachinfo_publish_info_.MatchMode=coach2robot_info.MatchMode;
    coachinfo_publish_info_.MatchType=coach2robot_info.MatchType;

    coachinfo_publish_info_.TestMode=coach2robot_info.TestMode;
    coachinfo_publish_info_.pointA.x=coach2robot_info.pointA.x_;
    coachinfo_publish_info_.pointA.y=coach2robot_info.pointA.y_;
    coachinfo_publish_info_.pointB.x=coach2robot_info.pointB.x_;
    coachinfo_publish_info_.pointB.y=coach2robot_info.pointB.y_;
    coachinfo_publish_info_.angleA=coach2robot_info.angleA;
    coachinfo_publish_info_.angleB=coach2robot_info.angleB;
    coachinfo_publish_info_.idA=coach2robot_info.id_A;
    coachinfo_publish_info_.idB=coach2robot_info.id_B;
    coachinfo_publish_info_.kickforce=coach2robot_info.kick_force;

    coach2robotinfo_pub_.publish(coachinfo_publish_info_);
}
void
Robot2coach::run()
{
    ros::spin();
}
