#include <stdio.h>
#include <signal.h>
#include <time.h>
#include "nubot/world_model/world_model.h"
using namespace nubot;
nubot::World_Model::World_Model(int argc,char** argv,const char * name)
{
    ros::init(argc,argv,name);
    const char * environment;
#ifdef SIMULATION
    std::string robot_name;
    std::string str = argv[1];
    std::string str2 = str.substr(str.size()-1);
    std::string robot_prefix = str.substr(0,str.size()-1);
    environment = str2.c_str();
    robot_name = str;
    ROS_FATAL("world_model: robot_name:%s",robot_name.c_str());
    nh = boost::make_shared<ros::NodeHandle>(robot_name);
    std::string info_topic = "/" + robot_prefix + "/receive_from_coach";
    std::string strategy_topic = "/" + robot_prefix + "/nubotcontrol/strategy";
    coach_sub_ = nh->subscribe(info_topic, 1 , &nubot::World_Model::receiveFromCoach, this);
#else
    std::string strategy_topic = "/nubotcontrol/strategy";
    /** 读取机器人标号并赋值. 在.bashrc中输入export AGENT=1，2，3，4，等等；*/
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return ;
    }
    nh = boost::make_shared<ros::NodeHandle>();
    /** RTDB通信模块的初始化，开辟内存空间*/
    if(DB_init() != 0)
    {
        ROS_WARN("RTDB没有成功初始化内存空间");
        return;
    }
#endif
    /** 订阅全向视觉节点topic，所有的机器人*/
    omin_vision_sub_    =  nh->subscribe("omnivision/OmniVisionInfo", 1 , &nubot::World_Model::updateOminivision, this);
    /** 订阅前向视觉节点topic，一般的移动机器人*/
    front_vision_sub_   =  nh->subscribe("front_vision/FrontBallInfo",1,&nubot::World_Model::updateFrontVision, this);
    /** 订阅kinect节点topic，守门员移动机器人*/
    kinect_vision_sub_  =  nh->subscribe("kinect/ballinfo",1,&nubot::World_Model::updateKinectBall, this);

    /** 接收来自策略等节点的信息*/
    strategy_info_sub_  =  nh->subscribe( strategy_topic, 10 , &nubot::World_Model::updateStrategyinfo, this);

    /** 发布更新的世界模型信息，包括感知信息，COACH信息等等 */
    worldmodelinfo_pub_ =  nh->advertise<nubot_common::WorldModelInfo>("worldmodel/worldmodelinfo",10);
    /** 30ms触发一次的定时器 */
    worldmodel_update_timer_ = nh->createTimer(ros::Duration(0.015),&World_Model::update,this);
    teammatesinfo_.resize(OUR_TEAM); //开辟内存空间；
    // 读取机器人标号，并赋值.
    AgentID_ = atoi(environment);
    teammatesinfo_[AgentID_-1].robot_info_.setID(AgentID_); //机器人ID标号设置；
    for(int i = 0; i<OUR_TEAM;i++)
       teammatesinfo_[AgentID_-1].robot_info_.setValid(false);
    obstacles_.setAgentID(AgentID_);
    teammateIDforBallSelected = -1;
    // 接收到的各种topic更新时间；
    omni_update_time_   = ros::Time::now();     // 机器人信息的更新时间，用于计算lifetime判断机器人信息的有效性；
    front_update_time_  = ros::Time::now();     // 足球信息的更新时间，用于计算lifetime判断足球信息的有效性；
    kinect_update_time_ = ros::Time::now();     // 障碍物信息的更新时间，用于计算lifetime判断障碍物信息的有效性；
    /** 当前仅仅传输五个机器人信息*/
    world_model_info_.robotinfo.resize(OUR_TEAM);
    receive_coach_count_ = ros::Time::now();;
    coach2robot_.MatchMode = STOPROBOT;
    coach2robot_.MatchType  = STOPROBOT;
}
nubot::World_Model::~World_Model(){
#ifndef SIMULATION
    DB_free();  /** RTDB通信模块的释放开辟内存空间*/
#endif
}

#ifdef SIMULATION
void
nubot::World_Model::receiveFromCoach(const nubot_common::CoachInfo & _coach)
{
//    coach2robot_.CoachStrategy = _coach.CoachStrategy ;
//    coach2robot_.CoachTactics = _coach.CoachTactics ;
//    coach2robot_.isPosition = _coach.isPosition ;
    coach2robot_.MatchMode = _coach.MatchMode ;
    coach2robot_.MatchType = _coach.MatchType ;
//    coach2robot_.MatchState = _coach.MatchState ;
//    coach2robot_.m_nCatchNumber = _coach.m_nCatchNumber ;
//    coach2robot_.m_nPassNumber = _coach.m_nPassNumber ;
//    for(int i = 0; i < 5; i++)
//        coach2robot_.target[i] =DPoint(_coach.target[i].x,_coach.target[i].y);
}
/**  仿真程序更新所有的机器人的策略信息，在实际比赛中通过RTDB进行传输，现在采用topic传输*/
void
nubot::World_Model::updateStrategyinfo(const nubot_common::simulation_strategy &_strategyinfo)
{
    int nums_robots = _strategyinfo.strategy_msgs.size();
    for(int i =0 ; i < nums_robots; i++)
    {
        nubot_common::StrategyInfo strategyinfo = _strategyinfo.strategy_msgs[i];
        int AgentId = strategyinfo.AgentID;
        if(AgentId < 1 || AgentId >OUR_TEAM)
            continue;
        PassCommands & pass_cmd_  = teammatesinfo_[AgentId-1].pass_cmds_;
        pass_cmd_.catchrobot_id   = strategyinfo.pass_cmd.catch_id;
        pass_cmd_.passrobot_id    = strategyinfo.pass_cmd.pass_id;
        pass_cmd_.is_dynamic_pass = strategyinfo.pass_cmd.is_dynamic_pass;
        pass_cmd_.is_static_pass  = strategyinfo.pass_cmd.is_static_pass;
        pass_cmd_.is_passout = strategyinfo.pass_cmd.is_passout;
        pass_cmd_.pass_pt    = DPoint(strategyinfo.pass_cmd.pass_pt.x,strategyinfo.pass_cmd.pass_pt.y);
        pass_cmd_.catch_pt   = DPoint(strategyinfo.pass_cmd.catch_pt.x,strategyinfo.pass_cmd.catch_pt.y);
        pass_cmd_.isvalid    = strategyinfo.pass_cmd.is_valid;
        teammatesinfo_[AgentId-1].robot_info_.setRolePreserveTime(strategyinfo.role_time);
        teammatesinfo_[AgentId-1].robot_info_.setCurrentRole(strategyinfo.role);
        teammatesinfo_[AgentId-1].robot_info_.setDribbleState(strategyinfo.is_dribble);
        teammatesinfo_[AgentId-1].robot_info_.setCurrentAction(strategyinfo.action);

        teammatesinfo_[AgentId-1].robot_info_.setTargetNum(1,strategyinfo.targetNum1);
        teammatesinfo_[AgentId-1].robot_info_.setTargetNum(2,strategyinfo.targetNum2);
        teammatesinfo_[AgentId-1].robot_info_.setTargetNum(3,strategyinfo.targetNum3);
        teammatesinfo_[AgentId-1].robot_info_.setTargetNum(4,strategyinfo.targetNum4);

        teammatesinfo_[AgentId-1].robot_info_.setcatchNum(strategyinfo.staticcatchNum);
        teammatesinfo_[AgentId-1].robot_info_.setpassNum(strategyinfo.staticpassNum);
    }
}
#else
//实际比赛中从nubot_control信息，写到teammatesinfo_中通过RTDB进行信息传输 更新策略得到的信息，包括发起的传球命令等等
void
nubot::World_Model::updateStrategyinfo(const nubot_common::StrategyInfo &strategyinfo)
{
    nubot_control_time_ = strategyinfo.header.stamp;
    PassCommands & pass_cmd_  = teammatesinfo_[AgentID_-1].pass_cmds_;
    pass_cmd_.catchrobot_id   = strategyinfo.pass_cmd.catch_id;
    pass_cmd_.passrobot_id    = strategyinfo.pass_cmd.pass_id;
    pass_cmd_.is_dynamic_pass = strategyinfo.pass_cmd.is_dynamic_pass;
    pass_cmd_.is_static_pass  = strategyinfo.pass_cmd.is_static_pass;
    pass_cmd_.is_passout = strategyinfo.pass_cmd.is_passout;
    pass_cmd_.pass_pt    = DPoint(strategyinfo.pass_cmd.pass_pt.x,strategyinfo.pass_cmd.pass_pt.y);
    pass_cmd_.catch_pt   = DPoint(strategyinfo.pass_cmd.catch_pt.x,strategyinfo.pass_cmd.catch_pt.y);
    pass_cmd_.isvalid    = strategyinfo.pass_cmd.is_valid;
    teammatesinfo_[AgentID_-1].robot_info_.setRolePreserveTime(strategyinfo.role_time);
    teammatesinfo_[AgentID_-1].robot_info_.setCurrentRole(strategyinfo.role);
    teammatesinfo_[AgentID_-1].robot_info_.setDribbleState(strategyinfo.is_dribble);
    teammatesinfo_[AgentID_-1].robot_info_.setCurrentAction(strategyinfo.action);

    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(1,strategyinfo.targetNum1);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(2,strategyinfo.targetNum2);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(3,strategyinfo.targetNum3);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(4,strategyinfo.targetNum4);

    teammatesinfo_[AgentID_-1].robot_info_.setcatchNum(strategyinfo.staticcatchNum);
    teammatesinfo_[AgentID_-1].robot_info_.setpassNum(strategyinfo.staticpassNum);
}
#endif
/**
    * 接收Kinect节点发布的消息，当前用于守门员机器人；
    * 更新世界模型中ball_info_，因为Kinect节点足球信息没有全局坐标，需要补充足球全局坐标系下坐标
    * @param [in] _ball_info：Kinect节点发布的消息，包含足球信息；
    */
void
nubot::World_Model::updateKinectBall(const nubot_common::BallInfo3d & _ball_info)
{
    kinect_update_time_ = _ball_info.header.stamp;
    Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
    BallObject kinect_ball;
    kinect_ball.setID(AgentID_);
    PPoint real_pt;
    bool pos_known = _ball_info.pos_known_2d||_ball_info.pos_known_3d;
    if(pos_known)
        real_pt = PPoint(DPoint(_ball_info.pos.x,_ball_info.pos.y));
    kinect_ball.setRealLocation(real_pt);
    PPoint pt(real_pt.angle_+robot_info.getHead(), real_pt.radius_);
    kinect_ball.setGlobalLocation(robot_info.getLocation()+DPoint(pt));
    /** 表示足球位置是否已知，是否检测到足球 */
    kinect_ball.setLocationKnown(pos_known);
    kinect_ball.setValid(false);  //这儿赋值位false，因为机器人还没有正常使用该信息；
    ball_info_.sensor_ball_[KINECT_BALL] = kinect_ball;
}
/**
    * 接收前向视觉节点发布的消息，当前用于非守门员机器人；
    * 更新世界模型中ball_info_，因为前向视觉节点足球信息没有全局坐标，需要补充足球全局坐标系下坐标
    * @param [in] _front_info：前向视觉节点发布的消息，包含足球信息；
    */
void
nubot::World_Model::updateFrontVision(const nubot_common::FrontBallInfo & _front_info)
{
    front_update_time_ = _front_info.header.stamp;
    Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
    BallObject frontvision_ball;
    frontvision_ball.setID(AgentID_);
    PPoint real_pt=PPoint(Angle(_front_info.real_pos.angle), _front_info.real_pos.radius);
    frontvision_ball.setRealLocation(real_pt);
    PPoint pt(real_pt.angle_+ robot_info.getHead(), real_pt.radius_);
    frontvision_ball.setGlobalLocation(robot_info.getLocation()+DPoint(pt));
    /** 表示足球位置是否已知，是否检测到足球 */
    frontvision_ball.setLocationKnown(_front_info.pos_known);
    frontvision_ball.setValid(true);
    frontvision_ball.setVelocityKnown(false);
    ball_info_.sensor_ball_[FRONT_BALL] = frontvision_ball;
    /** 球速前向视觉系统的球速*/
//为了保证球速的一致性，不使用前向球速
//    ball_info_.front_ball_record_.push_back(frontvision_ball);
//    ball_info_.front_ball_time_.push_back(front_update_time_);
//    static int  num_predict_errors=0;
//    bool is_start_again = ball_info_.evaluateVelocity(ball_info_.front_ball_record_,
//                                                      ball_info_.front_ball_time_, num_predict_errors);
//    ball_info_.sensor_ball_[FRONT_BALL] = ball_info_.front_ball_record_[ball_info_.front_ball_record_.size()-1];
//    /** 重新求取速度，则将所有数据清空*/
//    if(is_start_again)
//    {
//        ball_info_.front_ball_record_.clear();
//        ball_info_.front_ball_time_.clear();
//        num_predict_errors = 0;
//    }
}

/**
    * 接收全向视觉节点发布的消息，当前用于所有的机器人；
    * 更新世界模型中robot_info，ball_info_，以及obstacles_；
    * @param [in] omni_info：全向视觉节点发布的消息，包含有机器人定位信息、障碍物信息、足球信息等等
    */
void
nubot::World_Model::updateOminivision(const nubot_common::OminiVisionInfo & omni_info)
{
    omni_update_time_ = omni_info.header.stamp;
    /** 给机器人信息赋值，其中特别要说明的是setValid表示机器人的开关电状态，关闭总开关时false */
    int robot_nums = omni_info.robotinfo.size();
    for(int  i = 0 ; i < robot_nums; i++)
    {
        int AgentId = omni_info.robotinfo[i].AgentID;
        if(AgentId < 1 || AgentId >OUR_TEAM)
            continue;
        Robot & robot_info = teammatesinfo_[AgentId-1].robot_info_;
        robot_info.setID(AgentId);
        robot_info.setVelocity(DPoint2d(omni_info.robotinfo[i].vtrans.x,omni_info.robotinfo[i].vtrans.y));
        robot_info.setLocation(DPoint2d(omni_info.robotinfo[i].pos.x,omni_info.robotinfo[i].pos.y));
        robot_info.setW(omni_info.robotinfo[i].vrot);
        robot_info.setHead(Angle(omni_info.robotinfo[i].heading.theta));
        robot_info.setStuck(omni_info.robotinfo[i].isstuck);
        robot_info.setValid(omni_info.robotinfo[i].isvalid);
#ifdef SIMULATION
        robot_info.setValid(true);
#endif
    }
    /** 给障碍物信息赋值 */
    std::vector< ObstacleObject > obstacles;
    int length=omni_info.obstacleinfo.pos.size();
    obstacles.reserve(length);
    for(int i = 0 ;i < length ; i++)
    {
        ObstacleObject object_temp;
        DPoint pt(omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y);
        PPoint polar(Angle(omni_info.obstacleinfo.polar_pos[i].angle),
                     omni_info.obstacleinfo.polar_pos[i].radius);
        object_temp.setLocation(pt);
        object_temp.setPolarLocation(polar);
        obstacles.push_back(object_temp);
    }
    obstacles_.setOmniObstacles(obstacles,AgentID_);
    /** 给world_model中的ball_info_类中的omni_ball_变量赋值 */
    BallObject omni_ball;
    omni_ball.setGlobalLocation(DPoint(omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y));
    /** 表示足球位置是否已知，是否检测到足球 */
    omni_ball.setLocationKnown(omni_info.ballinfo.pos_known);
    omni_ball.setRealLocation(PPoint(Angle(omni_info.ballinfo.real_pos.angle),
                                     omni_info.ballinfo.real_pos.radius));
    omni_ball.setID(AgentID_);
    omni_ball.setValid(true);
#ifdef SIMULATION
    /** 仿真时候足球的velocity信息已知*/
    omni_ball.setVelocity(DPoint2d(omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.y));
    omni_ball.setVelocityKnown(omni_info.ballinfo.velocity_known);
    ball_info_.sensor_ball_[OMNI_BALL] = omni_ball;
#else
    ball_info_.omni_ball_record_.push_back(omni_ball);
    ball_info_.omni_ball_time_.push_back(omni_update_time_);
    static int  num_predict_errors=0;
    bool is_start_again = ball_info_.evaluateVelocity(ball_info_.omni_ball_record_,
                                                      ball_info_.omni_ball_time_, num_predict_errors);
    ball_info_.sensor_ball_[OMNI_BALL] = ball_info_.omni_ball_record_[ball_info_.omni_ball_record_.size()-1];
    /** 重新求取速度，则将所有数据清空*/
    if(is_start_again)
    {
        ball_info_.omni_ball_record_.clear();
        ball_info_.omni_ball_time_.clear();
        num_predict_errors = 0;
    }
#endif
 //   ROS_INFO("omni: %.f %.f",ball_info_.sensor_ball_[OMNI_BALL].getVelocity().x_,ball_info_.sensor_ball_[OMNI_BALL].getVelocity().y_);
}
/**
    * 周期更新世界模型（30ms，ROS定时器）
    * 因为接收的各种topic信息是基于消息中断的，世界模型无法直接判断是否数据已经更新
    * 采用定时器更新世界模型的信息，可以判断信息是否有用，足球、障碍物、机器人、队友是否处于正常状态（根据数据更新的时间戳）
    * 同时可以对所有信息进行融合，得到一个最终统一的世界模型
    */
void
nubot::World_Model::update(const ros::TimerEvent & )
{
    static ros::Time time_before = ros::Time::now();
    ros::Duration duration = ros::Time::now() - time_before;
    time_before = ros::Time::now();
    ROS_INFO("%.3f",duration.toNSec()/1000000.0);
    static  int  streaming_cout = 0;
#ifdef SIMULATION
    /** 所有的信息均认为有效，不需要额外的融合措施，仅仅需要将其发布到控制节点和COACH即可*/
    /** 直接填充障碍物信息*/
    std::vector< ObstacleObject> omni_obstacles ;
    obstacles_.getOmniObstacles(omni_obstacles,AgentID_);
    obstacles_.fuse_obs_.clear();
    obstacles_.self_obs_.clear();
    for(int i =0; i <omni_obstacles.size(); i++)
    {
        obstacles_.self_obs_.push_back(omni_obstacles[i].getLocation());
        bool isteammates =false;
        for(int j = 0 ; j< OUR_TEAM; j++)
        {
            nubot::Robot & robot_info = teammatesinfo_[j].robot_info_;
            if(robot_info.isValid())
            {
                double distance = robot_info.getLocation().distance(omni_obstacles[i].getLocation());
                if(distance < 10)
                    isteammates = true;
            }
        }
        if(!isteammates)
            obstacles_.fuse_obs_.push_back(omni_obstacles[i].getLocation());
    }

    /** 直接填充足球的信息，不需要融合算法,直接认为球信息完全已知，但是要更新队友球的信息*/
    ball_info_.fuse_ball_ = ball_info_.sensor_ball_[OMNI_BALL];
    teammatesinfo_[AgentID_-1].ball_info_ =ball_info_.fuse_ball_ ;
    ball_info_.ball_info_state_ = SEEBALLBYOWN;
    /** 将其他机器人的足球信息更新，特别是极坐标需要更新*/
    for(int i = 0 ; i < OUR_TEAM; i++)
    {
       nubot::Robot & robot_info = teammatesinfo_[i].robot_info_;
       if(robot_info.isValid() && i+1!=AgentID_)
       {
            BallObject ball_tmp = ball_info_.fuse_ball_;
            DPoint pt=ball_tmp.getGlobalLocation()-robot_info.getLocation();
            PPoint pts(pt);
            ball_tmp.setRealLocation(PPoint(pts.angle_-robot_info.getHead(),pts.radius_));
            teammatesinfo_[i].ball_info_ =ball_tmp;
       }
    }
    publish();
    //sendToCoach();
#else
    /**  接收来自coach的信息*/
    if(streaming_cout == 0 )
        DB_get(0,MESSAGEFROMCOACHINFO,&coach2robot_);    //0  暂时代表coach agent
    /** 更新nubot_control信息，判断其有效性,将有用的信息传输回去*/
    ros::Duration duration1 = ros::Time::now() - nubot_control_time_;
    double lifttime = duration1.toNSec()/1000000.0;
    if( lifttime > 0 && lifttime < NOT_DATAUPDATE)
        teammatesinfo_[AgentID_-1].pass_cmds_.isvalid =true;
    else
        teammatesinfo_[AgentID_-1].pass_cmds_.isvalid =false;
    /** 更新所有机器人信息，判断其有效性，采用RTDB时间戳,自身采用的是ROS::Time记录*/
    Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
    if(streaming_cout == 0 )
        updateInfo();
    /** 更新足球的信息*/
    if(teammateIDforBallSelected != -1)
    {
        /** 将足球的世界坐标转换到到当前机器人体坐标系下*/
        BallObject ball_tmp = teammatesinfo_[teammateIDforBallSelected].ball_info_;
        DPoint pt=ball_tmp.getGlobalLocation()-robot_info.getLocation();
        PPoint pts(pt);
        ball_tmp.setRealLocation(PPoint(pts.angle_-robot_info.getHead(),pts.radius_));
        ball_info_.update(ball_tmp,true);
    }
    else
        ball_info_.update(teammatesinfo_[0].ball_info_,false);
    /** 自身感知到的足球信息发布给队友，不能使用融合后的足球*/
    teammatesinfo_[AgentID_-1].ball_info_ = ball_info_.own_ball_;
    /** 更新自身检测到的障碍物信息*/
    for(size_t i = 0 ; i <OUR_TEAM ; i++)
        obstacles_.setRobotInfo(teammatesinfo_[i].robot_info_.getLocation(),teammatesinfo_[i].robot_info_.isValid(),i+1);
    obstacles_.update();
    /** 信息发布到上层控制节点*/
    publish();
    /** 信息发布到COACH*/
    //sendToCoach();            //实际并没使用
    //!  将自身的通过视觉检测到的原始障碍物信息，传输给队友，进行障碍物融合
    std::vector<ObstacleObject> omni_obs;
    obstacles_.getOmniObstacles(omni_obs,AgentID_);
    for(int i = 0 ; i < MAX_OBSNUMBER_CONST ; i++)
    {
        if(i < omni_obs.size())
            teammatesinfo_[AgentID_-1].obs_info_[i] = omni_obs[i];
        else
            teammatesinfo_[AgentID_-1].obs_info_[i].clear();
    }
    if(streaming_cout == 0)
    {
        /** 自身感知的信息发布给队友*/
        if(DB_put(TEAMMATESINFO, &teammatesinfo_[AgentID_-1]) == -1)
        {
            DB_free();
            ROS_ERROR("RTDB发送信息失败，请重启");
        }
        streaming_cout++;
    }
    else
        streaming_cout = 0;
#endif
}
/**
    * 发布世界模型信息到机器人上层控制节点（包含队友的信息）；
    * 自身机器人信息、障碍物信息、COACH信息以及队友信息（主要是定位信息、足球信息、策略信息）
    */
void
nubot::World_Model::publish()
{
    /**  将自身感知到的信息以及通信得到的队友信息，发送到上层控制节点*/
    world_model_info_.robotinfo.clear();
    world_model_info_.robotinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        Robot & robot_info = teammatesinfo_[i].robot_info_;
        world_model_info_.robotinfo[i].AgentID = robot_info.getID();

        world_model_info_.robotinfo[i].targetNum1=robot_info.getTargetNum(1);
        world_model_info_.robotinfo[i].targetNum2=robot_info.getTargetNum(2);
        world_model_info_.robotinfo[i].targetNum3=robot_info.getTargetNum(3);
        world_model_info_.robotinfo[i].targetNum4=robot_info.getTargetNum(4);
        world_model_info_.robotinfo[i].staticpassNum=robot_info.getpassNum();
        world_model_info_.robotinfo[i].staticcatchNum=robot_info.getcatchNum();

        world_model_info_.robotinfo[i].pos.x      = robot_info.getLocation().x_;
        world_model_info_.robotinfo[i].pos.y      = robot_info.getLocation().y_;
        world_model_info_.robotinfo[i].heading.theta = robot_info.getHead().radian_;
        world_model_info_.robotinfo[i].vtrans.x   = robot_info.getVelocity().x_;
        world_model_info_.robotinfo[i].vtrans.y   = robot_info.getVelocity().y_;
        world_model_info_.robotinfo[i].isstuck    = robot_info.isStuck();
        world_model_info_.robotinfo[i].iskick     = robot_info.isKickoff();
        world_model_info_.robotinfo[i].isvalid    = robot_info.isValid();
        world_model_info_.robotinfo[i].vrot       = robot_info.getW();
        world_model_info_.robotinfo[i].current_role = robot_info.getCurrentRole();
        world_model_info_.robotinfo[i].role_time =  robot_info.getRolePreserveTime();
        world_model_info_.robotinfo[i].target.x = robot_info.getTarget().x_;
        world_model_info_.robotinfo[i].target.y = robot_info.getTarget().y_;
        world_model_info_.robotinfo[i].isdribble = robot_info.getDribbleState();
    }
    /** 单机器人障碍物，主要用于避障*/
    std::vector< DPoint > tracker;
    obstacles_.getSelfObsTracker(tracker);
    world_model_info_.obstacleinfo.pos.clear();
    world_model_info_.obstacleinfo.pos.resize(tracker.size());
    for(std::size_t i = 0; i< tracker.size() ; i++)
    {
        nubot_common::Point2d point;
        point.x=tracker[i].x_;
        point.y=tracker[i].y_;
        world_model_info_.obstacleinfo.pos[i]= point;
    }
    /** 多机器人障碍物融合结果，主要用于防守跑位，盯人防守等*/
    std::vector< DPoint > opptracker;
    obstacles_.getFuseObsTracker(opptracker);
    world_model_info_.oppinfo.pos.clear();
    world_model_info_.oppinfo.pos.resize(opptracker.size());
    for(std::size_t i = 0; i< opptracker.size() ; i++)
    {
        nubot_common::Point2d point;
        point.x=opptracker[i].x_;
        point.y=opptracker[i].y_;
        world_model_info_.oppinfo.pos[i]= point;
    }

    /** 发布球的信息，自身发布融合之后的足球*/
    world_model_info_.ballinfo.clear();
    world_model_info_.ballinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        BallObject ball_info;
        if(i == AgentID_-1)
            ball_info=ball_info_.fuse_ball_;
        else
            ball_info = teammatesinfo_[i].ball_info_;
        world_model_info_.ballinfo[i].pos.x =  ball_info.getGlobalLocation().x_;
        world_model_info_.ballinfo[i].pos.y =  ball_info.getGlobalLocation().y_;
        world_model_info_.ballinfo[i].real_pos.angle = ball_info.getRealLocation().angle_.radian_;
        world_model_info_.ballinfo[i].real_pos.radius = ball_info.getRealLocation().radius_;
        world_model_info_.ballinfo[i].velocity.x = ball_info.getVelocity().x_;
        world_model_info_.ballinfo[i].velocity.y = ball_info.getVelocity().y_;
        world_model_info_.ballinfo[i].velocity_known = ball_info.isVelocityKnown();
        world_model_info_.ballinfo[i].pos_known      = ball_info.isLocationKnown();
        if(i == AgentID_-1)
            world_model_info_.ballinfo[i].ballinfostate  = ball_info_.ball_info_state_;
    }

    /** 发布coach的信息*/
    world_model_info_.coachinfo.MatchMode =coach2robot_.MatchMode;
    world_model_info_.coachinfo.MatchType =coach2robot_.MatchType;

    /**  传输的初始值*/
    world_model_info_.pass_cmd.catch_id = -1;
    world_model_info_.pass_cmd.pass_id = -1;
    world_model_info_.pass_cmd.is_dynamic_pass  = false;
    world_model_info_.pass_cmd.is_static_pass  = false;
    world_model_info_.pass_cmd.is_passout = false;
    world_model_info_.pass_cmd.is_valid = false;
    //！ 传输有传接球命令的消息，到机器人
    for(int i = 0 ; i < OUR_TEAM; i++)
    {
        Teammatesinfo & teammates = teammatesinfo_[i];
        if(i != AgentID_-1)
        {
            //!表示当前机器人是需要接球的机器人,通过该信息判断自身的状态，所有机器人的都能收到传接球指令
            if(teammates.pass_cmds_.isvalid &&
                    (teammates.pass_cmds_.is_static_pass||teammates.pass_cmds_.is_dynamic_pass||teammates.pass_cmds_.is_passout))
            {
                world_model_info_.pass_cmd.catch_id = teammates.pass_cmds_.catchrobot_id;
                world_model_info_.pass_cmd.pass_id  = teammates.pass_cmds_.passrobot_id;
                world_model_info_.pass_cmd.is_dynamic_pass  = teammates.pass_cmds_.is_dynamic_pass;
                world_model_info_.pass_cmd.is_static_pass   = teammates.pass_cmds_.is_static_pass;
                world_model_info_.pass_cmd.is_passout  = teammates.pass_cmds_.is_passout;
                world_model_info_.pass_cmd.is_valid    = teammates.pass_cmds_.isvalid;
                world_model_info_.pass_cmd.pass_pt.x   = teammates.pass_cmds_.pass_pt.x_;
                world_model_info_.pass_cmd.pass_pt.y   = teammates.pass_cmds_.pass_pt.y_;
                world_model_info_.pass_cmd.catch_pt.x  = teammates.pass_cmds_.catch_pt.x_;
                world_model_info_.pass_cmd.catch_pt.y  = teammates.pass_cmds_.catch_pt.y_;
            }
        }
    }
    worldmodelinfo_pub_.publish(world_model_info_);
}
/**
    * 周期更新队友以及COACH信息（30ms，ROS定时器，COACH等待移植到UBUNTU）
    */
void
nubot::World_Model::updateInfo()            // if the simulation flag is set, this function will not be run
{
    /** 更新所有队友的信息，判断这些信息是否有效，根据时间戳*/
    double min_distance_ball = 100000000;
    teammateIDforBallSelected = -1;
    DPoint ball_vec = DPoint(0,0);
    bool   ball_vec_known = false;
    for( int i = 0 ; i < OUR_TEAM ; i++ )
    {
        if( AgentID_ != i+1 )
        {
            int ltime = DB_get(i+1, TEAMMATESINFO, &teammatesinfo_[i]);
            //接收到的队友信息，并记录其间隔时间，可能表示信息无效；
            teammatesinfo_[i].robot_info_.setlifetime(ltime);
            teammatesinfo_[i].ball_info_.setlifetime(ltime);
            teammatesinfo_[i].robot_info_.update();

            /** 选择机器人于足球最近的作为队友看到的足球，最终会与自身感知到的足球融合*/
            /** 通信没有中断（ltime）并且在获得足球信息的机器人上topic没有延迟太长时间（isValid()）*/
            if(ltime > 0  && ltime < NOT_DATAUPDATE     &&
                    teammatesinfo_[i].ball_info_.isValid()  &&
                    teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].ball_info_.setValid(true);
            else
                teammatesinfo_[i].ball_info_.setValid(false);

            if( teammatesinfo_[i].ball_info_.isValid() &&  teammatesinfo_[i].ball_info_.isLocationKnown())
            {
                /** 机器人与足球之间的距离，选出其中距离最短的作为当前机器人于足球的距离*/
                if(!ball_vec_known && teammatesinfo_[i].ball_info_.isVelocityKnown()) //
                {
                    ball_vec_known = true;
                    ball_vec = teammatesinfo_[i].ball_info_.getVelocity();
                }
                if(min_distance_ball >  teammatesinfo_[i].ball_info_.getRealLocation().radius_)
                {
                    min_distance_ball = teammatesinfo_[i].ball_info_.getRealLocation().radius_;
                    teammateIDforBallSelected = i;
                    if(teammatesinfo_[i].ball_info_.isVelocityKnown())
                    {
                        ball_vec_known = true;
                        ball_vec = teammatesinfo_[i].ball_info_.getVelocity();
                    }
                }
            }
            //!判断读取的传接球信息是否有效
            if(ltime > 0  && ltime < NOT_DATAUPDATE  &&
                    teammatesinfo_[i].pass_cmds_.isvalid &&
                    teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].pass_cmds_.isvalid = true;
            else
                teammatesinfo_[i].pass_cmds_.isvalid = false;

            /** 写入障碍物的信息, 必须通信已经建立,将障碍物写入到*/
            std::vector<ObstacleObject> obs_info;
            if(ltime > 0  && ltime < NOT_DATAUPDATE  &&
                    teammatesinfo_[i].robot_info_.isValid())
            {
                for(int j = 0 ; j <MAX_OBSNUMBER_CONST;j++)
                {
                    if(teammatesinfo_[i].obs_info_[j].getPolarLocation().radius_ != 10000)
                        obs_info.push_back(teammatesinfo_[i].obs_info_[j]);
                }
                obstacles_.setOmniObstacles(obs_info,i+1);
            }
            else
                obstacles_.clearOmniObstacles(i+1);
        }
    }

    if(teammateIDforBallSelected!=-1)
    {
        teammatesinfo_[teammateIDforBallSelected].ball_info_.setVelocityKnown(ball_vec_known);
        teammatesinfo_[teammateIDforBallSelected].ball_info_.setVelocity(ball_vec);
    }
    /** 根据接收到的topic更新自身的信息,转换为ms*/

    ros::Time nowtime =  ros::Time::now();
    ros::Duration duration = nowtime - omni_update_time_;
    int self_time = duration.toNSec()/1000000.0;
    teammatesinfo_[AgentID_-1].robot_info_.setlifetime(self_time); //time -> ms
    if(self_time < 0 || self_time > NOT_DATAUPDATE)
        obstacles_.clearOmniObstacles(AgentID_);

    /** 更新全向视觉节点接收topic时间到当前的时间间隔，足球信息*/
    ball_info_.sensor_ball_[OMNI_BALL].setlifetime(self_time);

    /** 更新前向视觉节点接收topic时间到当前的时间间隔，足球信息*/
    duration = nowtime - front_update_time_;
    self_time = duration.toNSec()/1000000.0;
    ball_info_.sensor_ball_[FRONT_BALL].setlifetime(self_time);

    /** 更新深度相机节点接收topic时间到当前的时间间隔，足球信息*/
    duration = nowtime -kinect_update_time_;
    self_time = duration.toNSec()/1000000.0;
    ball_info_.sensor_ball_[KINECT_BALL].setlifetime(self_time);
    /**  下面可能加上障碍物，用于障碍物融合*/

    teammatesinfo_[AgentID_-1].robot_info_.update();
}

/*void
nubot::World_Model::sendToCoach()
{
    Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
    robot2coach_.robot_id  =robot_info.getID();
    robot2coach_.robot_head=robot_info.getHead().radian_;
    robot2coach_.robot_global_loc.x=robot_info.getLocation().x_;
    robot2coach_.robot_global_loc.y=robot_info.getLocation().y_;

    robot2coach_.ball_global_loc.x=ball_info_.own_ball_.getGlobalLocation().x_;
    robot2coach_.ball_global_loc.y=ball_info_.own_ball_.getGlobalLocation().y_;
    DPoint ball_real(ball_info_.own_ball_.getRealLocation());
    robot2coach_.ball_real_loc.x=ball_real.x_;
    robot2coach_.ball_real_loc.y=ball_real.y_;
    robot2coach_.ball_global_vec.x = ball_info_.own_ball_.getVelocity().x_;
    robot2coach_.ball_global_vec.y = ball_info_.own_ball_.getVelocity().y_;
    robot2coach_.ball_vec_known = ball_info_.own_ball_.isVelocityKnown();
    robot2coach_.ball_loc_known = ball_info_.own_ball_.isLocationKnown();

    std::vector<DPoint> tracker;
    obstacles_.getFuseObsTracker(tracker);
    for(int i=0; i < MAX_OBSTALCES_NUMS_CONST; i++)
    {
        if (i<tracker.size())
        {
            robot2coach_.otherRobot[i].x=tracker[i].x_;
            robot2coach_.otherRobot[i].y=tracker[i].y_;
        }
        else
        {
            robot2coach_.otherRobot[i].x=-10000;
            robot2coach_.otherRobot[i].y=-10000;
        }
    }

    robot2coach_.is_kick_off= robot_info.isKickoff();
    robot2coach_.is_dribble = robot_info.getDribbleState();
    robot2coach_.role       = robot_info.getCurrentRole();
    robot2coach_.action     = robot_info.getCurrentAction();
    robot2coach_.is_valid   = robot_info.isValid();
#ifdef SIMULATION
    static int buffer=sizeof(MessageToCoach);
    if (sendData(coach_socket_,&robot2coach_,buffer) != buffer)
        printf("this data is error");
#else
    DB_put(MESSAGETOACHINFO, &robot2coach_);
#endif
}*/
