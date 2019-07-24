#include <stdio.h>
#include <signal.h>
#include <time.h>
#include "world_model/world_model.h"

using namespace nubot;
using namespace std;
nubot::World_Model::World_Model(int argc,char** argv,const char * name)
{
    ros::init(argc,argv,name);
    const char * environment;
#ifdef SIMULATION
    std::string robot_name = argv[1];
    std::string num = robot_name.substr(robot_name.size()-1);
    std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
    environment = num.c_str();
    ROS_FATAL("world_model: robot_name:%s",robot_name.c_str());
    nh = boost::make_shared<ros::NodeHandle>(robot_name);
    std::string info_topic = "/" + robot_prefix + "/receive_from_coach";
    std::string strategy_topic = "/" + robot_prefix + "/nubotcontrol/strategy";
    coach_sub_ = nh->subscribe(info_topic, 1 , &nubot::World_Model::receiveFromCoach, this);
#else
    std::string strategy_topic = "/nubotcontrol/strategy";
    /// 读取机器人标号并赋值. 在.bashrc中输入export AGENT=1，2，3，4，等等
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return ;
    }
    nh = boost::make_shared<ros::NodeHandle>();
    /// RTDB通信模块的初始化，开辟内存空间
    if(DB_init() != 0)
    {
        ROS_WARN("RTDB没有成功初始化内存空间");
        return;
    }
#endif
    /// 订阅自身全向视觉节点topic
    omin_vision_sub_    =  nh->subscribe("omnivision/OmniVisionInfo",1,&nubot::World_Model::updateOminivision, this);
    /// 订阅自身kinect节点topic
    kinect_vision_sub_  =  nh->subscribe("/ball_obstacle_position",1,&nubot::World_Model::updateKinectBall, this);
    /// 订阅自身nubot_control节点topic
    strategy_info_sub_  =  nh->subscribe(strategy_topic, 10 , &nubot::World_Model::updateStrategyinfo, this);
    /// 发布经过RTDB更新后的世界模型信息，包括来自队友的信息，COACH信息*/
    worldmodelinfo_pub_ =  nh->advertise<nubot_common::WorldModelInfo>("worldmodel/worldmodelinfo",10);
    /// 30ms触发一次的定时器
    worldmodel_update_timer_ = nh->createTimer(ros::Duration(0.015),&World_Model::update,this);

    /// 开辟内存空间,用于存放RTDB中的队友信息
    teammatesinfo_.resize(OUR_TEAM);
    AgentID_ = atoi(environment);
    teammatesinfo_[AgentID_-1].robot_info_.setID(AgentID_);
    for(int i = 0; i<OUR_TEAM;i++)
       teammatesinfo_[AgentID_-1].robot_info_.setValid(false);
    teammateIDforBallSelected = -1;

    /// 存放该机器人识别到的障碍物
    obstacles_.setAgentID(AgentID_);

    /// 初始化各个节点topic的更新时间,用于计算lifetime判断该信息的有效性
    omni_update_time_   = ros::Time::now();
    front_update_time_  = ros::Time::now();
    kinect_update_time_ = ros::Time::now();
    /// 记录coach信息的更新时间,coach信息来自于RTDB
    receive_coach_count_= ros::Time::now();
    coach2robot_.MatchMode = STOPROBOT;
    coach2robot_.MatchType  = STOPROBOT;

    /// world_model_info是作为向control节点传输世界模型信息的msg类型
    world_model_info_.robotinfo.resize(OUR_TEAM);    
}
nubot::World_Model::~World_Model(){
#ifndef SIMULATION
    /// RTDB通信模块的释放开辟内存空间
    DB_free();
#endif
}

#ifdef SIMULATION
void
nubot::World_Model::receiveFromCoach(const nubot_common::CoachInfo & _coach)
{
    coach2robot_.MatchMode  = _coach.MatchMode ;
    coach2robot_.MatchType  = _coach.MatchType ;
    coach2robot_.TestMode   = _coach.TestMode;
    coach2robot_.pointA.x_     = _coach.pointA.x;
    coach2robot_.pointA.y_     = _coach.pointA.y;
    coach2robot_.pointB.x_     = _coach.pointB.x;
    coach2robot_.pointB.y_     = _coach.pointB.y;
    coach2robot_.angleA     = _coach.angleA;
    coach2robot_.angleB     = _coach.angleB;
    coach2robot_.id_A       = _coach.idA;
    coach2robot_.id_B       = _coach.idB;
    coach2robot_.kick_force = _coach.kickforce;
//    ROS_INFO("matchmode:%d",coach2robot_.MatchMode);
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

/// \brief 从nubot_control得到的信息，写到teammatesinfo_中通过RTDB进行信息传输
/// \param[in] 来自nubot_control的策略信息strategyinfo
void
nubot::World_Model::updateStrategyinfo(const nubot_common::StrategyInfo &strategyinfo)
{
    /// 记录更新时间
    nubot_control_time_ = strategyinfo.header.stamp;

    /// 传接球信息(pass_cmd待修改)
    PassCommands & pass_cmd_  = teammatesinfo_[AgentID_-1].pass_cmds_;
    pass_cmd_.catchrobot_id   = strategyinfo.pass_cmd.catch_id;
    pass_cmd_.passrobot_id    = strategyinfo.pass_cmd.pass_id;
    pass_cmd_.is_dynamic_pass = strategyinfo.pass_cmd.is_dynamic_pass;
    pass_cmd_.is_static_pass  = strategyinfo.pass_cmd.is_static_pass;
    pass_cmd_.is_passout = strategyinfo.pass_cmd.is_passout;
    pass_cmd_.pass_pt    = DPoint(strategyinfo.pass_cmd.pass_pt.x,strategyinfo.pass_cmd.pass_pt.y);
    pass_cmd_.catch_pt   = DPoint(strategyinfo.pass_cmd.catch_pt.x,strategyinfo.pass_cmd.catch_pt.y);
    pass_cmd_.isvalid    = strategyinfo.pass_cmd.is_valid;

    /// 当前机器人状态
    teammatesinfo_[AgentID_-1].robot_info_.setRolePreserveTime(strategyinfo.role_time);
    teammatesinfo_[AgentID_-1].robot_info_.setCurrentRole(strategyinfo.role);
    teammatesinfo_[AgentID_-1].robot_info_.setDribbleState(strategyinfo.is_dribble);
    teammatesinfo_[AgentID_-1].robot_info_.setCurrentAction(strategyinfo.action);

    /// 静态发球时各个机器人的站位分配以及传接球机器人编号
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(1,strategyinfo.targetNum1);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(2,strategyinfo.targetNum2);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(3,strategyinfo.targetNum3);
    teammatesinfo_[AgentID_-1].robot_info_.setTargetNum(4,strategyinfo.targetNum4);
    teammatesinfo_[AgentID_-1].robot_info_.setcatchNum(strategyinfo.staticcatchNum);
    teammatesinfo_[AgentID_-1].robot_info_.setpassNum(strategyinfo.staticpassNum);
}
#endif

/// \brief 接收Kinect节点发布的消息,当前Kinect信息由TX1处理后再传到工控机
///        更新世界模型中ball_info_,因为Kinect节点足球信息没有全局坐标，需要补充足球全局坐标系下坐标
/// \param[in] _tx1_info：Kinect节点发布的消息，包含足球信息；

void
nubot::World_Model::updateKinectBall(const nubot_common::object_info & tx1_info)
{
   bool ball_know;
   bool obs_know;

   /// 接收到的TX1是否检测到球与障碍的标志位
   ball_know=tx1_info.ball_know;
   obs_know=tx1_info.obs_know;

   /// 更新接收到信息的时间
   kinect_update_time_=ros::Time::now();
   /// 指向本机器人信息的内存空间
   Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
   /// Kinect识别的球信息
   BallObject kinect_ball;
   kinect_ball.setID(AgentID_);

   PPoint real_pt;
   /// 机器人的世界坐标
   DPoint robot_loc=robot_info.getLocation();
   /// 是否检测到有效球的标志
   int flag=0;
   /// 检测到障碍物的个数，为了之后给障碍物开辟空间
   int length;
   /// 判断Kinect的信息是否有效,初始为true
   kinect_ball.setValid(true);

   /// 如果TX1检测到球
   if(ball_know)
   {
       float distance;
       /// 检测到几个球进行循环处理
       for (int k=0;k<tx1_info.ball_pos.size();k++)
       {
           /// 除以10，是将ms单位转换为cm， 将球xy坐标（已经是相对于机器人的了）转换为相对于机器人的极坐标
           real_pt=PPoint(DPoint(tx1_info.ball_pos[k].x/10.0,tx1_info.ball_pos[k].y/10.0));
           PPoint pt(real_pt.angle_+robot_info.getHead(), real_pt.radius_);
           DPoint w_pt(robot_loc+DPoint(pt));
           printf("ball word %f %f\n",w_pt.x_,w_pt.y_);
           /// 判断足球是否在场地上
           if (field_info_.isInInterField(w_pt))
           {
               /// 如果在场地上，且第一次检测到有效球，初始化kinect_ball与distance
               if (flag==0)
               {
                   distance=pt.radius_;
                   flag=1;
                   /// 初始化kinect ball的世界坐标(xy)，相对与robot的极坐标
                   kinect_ball.setGlobalLocation(w_pt);
                   kinect_ball.setRealLocation(real_pt);
               }
               /// 如果在场地上，但不是第一个检测到的有效球，比较哪个球的距离更近
               else
               {
                   if(pt.radius_<distance)
                   {
                       kinect_ball.setGlobalLocation(w_pt);
                       kinect_ball.setRealLocation(real_pt);
                   }
               }
           }
       }
       /// 如果前面已经检测到了有效球
       if (flag==1)
       {
           kinect_ball.setLocationKnown(true);
           /// 更新world model中ball的kinect信息
           ball_info_.sensor_ball_[KINECT_BALL] = kinect_ball;
       }
       /// tx1检测到的球不在场地范围内
       else
       {
           /// 这种情况，将之前的部分标志重置
           ball_know=false;
           kinect_ball.setLocationKnown(false);
           ball_info_.sensor_ball_[KINECT_BALL] = kinect_ball;
       }
   }
   /// tx1没有检测到球
   else
   {
       kinect_ball.setLocationKnown(false);
       ball_info_.sensor_ball_[KINECT_BALL] = kinect_ball;
   }
   //ROS_INFO("KINECT BALL POSITION %d %f %d %d \n",kinect_ball.getRealLocation().angle_.degree(),kinect_ball.getRealLocation().radius_,kinect_ball.isValid());

   /// 如果TX1检测到障碍物
   if(obs_know)
   {
       /// 根据TX1检测到的障碍物个数开辟内存空间
       std::vector< ObstacleObject > obstacles;
       length=tx1_info.obs_pos.size();
       obstacles.reserve(length);

       for(int i = 0 ;i < length; i++)
       {
           /// 用于存放临时障碍物
           ObstacleObject object_temp;
           PPoint r_p=PPoint(DPoint(tx1_info.obs_pos[i].x/10.0,tx1_info.obs_pos[i].y/10.0));
           /// 将障碍物的相对坐标转化为世界坐标
           DPoint pt(tx1_info.obs_pos[i].x/10.0,tx1_info.obs_pos[i].y/10.0);
           PPoint ppt(r_p.angle_+robot_info.getHead(), r_p.radius_);
           DPoint w_pt(robot_loc+DPoint(ppt));
           //ROS_INFO("world coodinate! %f %f %f %f\n",tx1_info.obs_pos[i].x/10.0,tx1_info.obs_pos[i].y/10.0,w_pt.x_, w_pt.y_);

           object_temp.setLocation(w_pt);
           object_temp.setPolarLocation(r_p);
           /// 将临时障碍物信息压入obstacles中
           obstacles.push_back(object_temp);
           /// 这里没有对障碍物是否在场地内进行判断
           /// 这里障碍物也没有什么setLocationKnown，setValid等参数设置
           /// 相当于是将TX1检测到的障碍物全部压到obstacles里面
       }
       /// 将Kinect检测到的所有障碍物信息压入world model中
       obstacles_.setKinectObstacles(obstacles,AgentID_);
   }
}


/// \brief 接收全向视觉节点发布的消息，当前用于所有的机器人；
/// 更新世界模型中robot_info，ball_info_，以及obstacles_；
/// @param [in] omni_info：全向视觉节点发布的消息，包含有机器人定位信息、障碍物信息、足球信息等等

void
nubot::World_Model::updateOminivision(const nubot_common::OminiVisionInfo & omni_info)
{
    /// 记录本次topic的更新时间
    omni_update_time_ = omni_info.header.stamp;
    /// 给机器人信息赋值，其中特别要说明的是setValid表示机器人的开关电状态，关闭总开关时false
    int robot_nums = omni_info.robotinfo.size();
    for(int  i = 0 ; i < robot_nums; i++)
    {
        int AgentId = omni_info.robotinfo[i].AgentID;
        if(AgentId < 1 || AgentId >OUR_TEAM)
            continue;
        /// 将omni_info放入teammatesinfo中对应编号的位置
        Robot & robot_info = teammatesinfo_[AgentId-1].robot_info_;
        robot_info.setID(AgentId);
        robot_info.setVelocity(DPoint2d(omni_info.robotinfo[i].vtrans.x,omni_info.robotinfo[i].vtrans.y));
        robot_info.setLocation(DPoint2d(omni_info.robotinfo[i].pos.x,omni_info.robotinfo[i].pos.y));
        robot_info.setW(omni_info.robotinfo[i].vrot);
        robot_info.setHead(Angle(omni_info.robotinfo[i].heading.theta));
        robot_info.setStuck(omni_info.robotinfo[i].isstuck);
        robot_info.setValid(omni_info.robotinfo[i].isvalid);
    }
    /// 给障碍物信息赋值
    std::vector< ObstacleObject > obstacles;
    int length=omni_info.obstacleinfo.pos.size();
    obstacles.reserve(length);
    for(int i = 0 ;i < length ; i++)
    {
        ObstacleObject object_temp;
        DPoint pt(omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y);
        PPoint polar(Angle(omni_info.obstacleinfo.polar_pos[i].angle),omni_info.obstacleinfo.polar_pos[i].radius);
        object_temp.setLocation(pt);
        object_temp.setPolarLocation(polar);
        obstacles.push_back(object_temp);
    }
    obstacles_.setOmniObstacles(obstacles,AgentID_);
    /// 给球信息赋值, world_model中的ball_info_类中有多个球信息, 这里针对omni_ball_
    BallObject omni_ball;
    omni_ball.setGlobalLocation(DPoint(omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y));
    omni_ball.setLocationKnown(omni_info.ballinfo.pos_known);
    omni_ball.setRealLocation(PPoint(Angle(omni_info.ballinfo.real_pos.angle),omni_info.ballinfo.real_pos.radius));
    omni_ball.setID(AgentID_);
    omni_ball.setValid(true);
#ifdef SIMULATION
    /// 仿真时候足球的velocity信息已知
    omni_ball.setVelocity(DPoint2d(omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.y));
    omni_ball.setVelocityKnown(omni_info.ballinfo.velocity_known);
    ball_info_.sensor_ball_[OMNI_BALL] = omni_ball;
#else
    ball_info_.omni_ball_record_.push_back(omni_ball);
    ball_info_.omni_ball_time_.push_back(omni_update_time_);
    static int  num_predict_errors=0;
    bool is_start_again = ball_info_.evaluateVelocity(ball_info_.omni_ball_record_,ball_info_.omni_ball_time_, num_predict_errors);
    ball_info_.sensor_ball_[OMNI_BALL] = ball_info_.omni_ball_record_[ball_info_.omni_ball_record_.size()-1];

    /// 重新求取速度，则将所有数据清空
    if(is_start_again)
    {
        ball_info_.omni_ball_record_.clear();
        ball_info_.omni_ball_time_.clear();
        num_predict_errors = 0;
    }
#endif
//    ROS_INFO("omni: %.f %.f",ball_info_.sensor_ball_[OMNI_BALL].getVelocity().x_,ball_info_.sensor_ball_[OMNI_BALL].getVelocity().y_);
}

/// \brief 周期更新世界模型（30ms，ROS定时器）
/// 因为接收的各种topic信息是基于消息中断的，世界模型无法直接判断是否数据已经更新
/// 采用定时器更新世界模型的信息，可以判断信息是否有用，足球、障碍物、机器人、队友是否处于正常状态（根据数据更新的时间戳）
/// 同时可以对所有信息进行融合，得到一个最终统一的世界模型
/// update函数本身很复杂混乱,在这里简要解释,他涉及到两方面的信息交互,一是将RTDB更新的信息融合(有的没有融合)后用于本机器人的上层节点(nubot_control)使用
/// 二是将本机器人上层节点(kinect_info,nubot_control,omni_vision)得到的信息压入teammatesinfo的对应区域,给其他机器人使用
/// 补充一点,所有融合的信息都是归本地使用,通过RTDB传输的信息都是未经融合的(包括障碍物和球)

void
nubot::World_Model::update(const ros::TimerEvent & )
{
    /// 输出两次定时函数间的实际间隔
    static ros::Time time_before = ros::Time::now();
    ros::Duration duration = ros::Time::now() - time_before;
    time_before = ros::Time::now();
//    ROS_INFO("duration is %.3f",duration.toNSec()/1000000.0);
    static int streaming_cout = 0;

#ifdef SIMULATION
    /// 所有的信息均认为有效，不需要额外的融合措施，仅仅需要将其发布到控制节点和COACH即可
    /// 直接填充障碍物信息
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

    /// 直接填充足球的信息，不需要融合算法,直接认为球信息完全已知，但是要更新队友球的信息
    ball_info_.fuse_ball_ = ball_info_.sensor_ball_[OMNI_BALL];
    teammatesinfo_[AgentID_-1].ball_info_ =ball_info_.fuse_ball_ ;
    ball_info_.ball_info_state_ = SEEBALLBYOWN;
    /// 将其他机器人的足球信息更新，特别是极坐标需要更新
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
    /// 接收来自coach的信息(自己使用),coach在RTDB中名为Agent0,信息格式是MASSAGEFROMCOACHINFO,没有判断信息的有效性
    if(streaming_cout==0)
        DB_get(0,MESSAGEFROMCOACHINFO,&coach2robot_);

    /// 判断来自nubot_control节点的信息(其他机器人使用)，判断其有效性,nubot_control节点中主要是传球信息
    ros::Duration duration1 = ros::Time::now() - nubot_control_time_;
    double lifttime = duration1.toNSec()/1000000.0;
    if( lifttime > 0 && lifttime < NOT_DATAUPDATE)
        teammatesinfo_[AgentID_-1].pass_cmds_.isvalid =true;
    else
        teammatesinfo_[AgentID_-1].pass_cmds_.isvalid =false;

    /// 从RTDB更新所有机器人信息(自己使用)，判断其有效性，采用RTDB时间戳,自身采用的是ROS::Time记录
    Robot & robot_info = teammatesinfo_[AgentID_-1].robot_info_;
    if(streaming_cout == 0 )
        updateInfo();

    /// 从RTDB更新所有机器人信息后,因为每个机器人都上传了多个球信息,需要进行融合
    /// teammateIDforBallSelected表示当前整个队伍选择的以谁为准的足球信息(自己使用)
    if(teammateIDforBallSelected != -1)
    {
        /// 将足球的世界坐标转换到到当前机器人体坐标系下
        BallObject ball_tmp = teammatesinfo_[teammateIDforBallSelected].ball_info_;
        DPoint pt=ball_tmp.getGlobalLocation()-robot_info.getLocation();
        PPoint pts(pt);
        ball_tmp.setRealLocation(PPoint(pts.angle_-robot_info.getHead(),pts.radius_));
        /// 将得到足球信息进行融合
        ball_info_.update(ball_tmp,true);
    }
    /// 如果没选出最合理的足球信息,以守门员看到的为准,即Agent=0
    else
    {
        ball_info_.update(teammatesinfo_[0].ball_info_,false);
    }

    /// 自身感知到的足球信息存入teammatesinfo中的对应位置(其他机器人使用)
    teammatesinfo_[AgentID_-1].ball_info_ = ball_info_.own_ball_;

    /// 更新自身检测到的障碍物信息(其他机器人使用)
    /// 从障碍物中滤除本队机器人
    for(size_t i = 0 ; i <OUR_TEAM ; i++)
        obstacles_.setRobotInfo(teammatesinfo_[i].robot_info_.getLocation(),teammatesinfo_[i].robot_info_.isValid(),i+1);
    /// 障碍物信息来自两个传感器
    std::vector<ObstacleObject> omni_obs;
    obstacles_.getOmniObstacles(omni_obs,AgentID_);
    std::vector<ObstacleObject> kinect_obs;
    obstacles_.getKinectObstacles(kinect_obs,AgentID_);

    /// Kinect检测到了障碍物
    if (kinect_obs.size()>0)
    {
        /// 全向障碍物信息融合kinect障碍物信息
        std::vector<ObstacleObject> omni_obs_after_fuse;
        /// 将Kinect的一定范围障碍物放入omni_obs_after_fuse
        for (int k=0;k<kinect_obs.size();k++)
        {
            PPoint polar_posi=kinect_obs[k].getPolarLocation();
            /// 下面判断位置要用的是相对位置，所以不同于全向，setlocation放的是全局位置，我在前面kinect回调函数中放的相对位置，下面pushback要放全局位置。
            if ((polar_posi.angle_.degree()<29)&&(polar_posi.angle_.degree()>-34)&&(polar_posi.radius_<610))
            {
                omni_obs_after_fuse.push_back(kinect_obs[k]);
             //   ROS_INFO("KINECT OBSTACLE %d %f\n",polar_posi.angle_.degree(),polar_posi.radius_);
            }
        }
        /// 将全向的一定范围障碍物放入omni_obs_after_fuse
        for (int o=0;o<omni_obs.size();o++)
        {
            PPoint polar_posi=omni_obs[o].getPolarLocation();
            if ((polar_posi.angle_.degree()>32)||(polar_posi.angle_.degree()<-36)||(polar_posi.angle_.degree()>-32 && polar_posi.angle_.degree()<29 && polar_posi.radius_>=610))
            {
                omni_obs_after_fuse.push_back(omni_obs[o]);
              //  ROS_INFO("OMNI OBSTACLE %d %f\n",polar_posi.angle_.degree(),polar_posi.radius_);
            }
        }
        /// 如果kinect没有检测到障碍物，则不会进入这个if条件，omni是全向的信息，没有融合kinect的信息
        omni_obs.clear();
        omni_obs=omni_obs_after_fuse;
        omni_obs_after_fuse.clear();
    }
    /// 利用卡尔曼滤波进行障碍物信息融合,得到的信息用于本机器人
    obstacles_.setOmniObstacles(omni_obs,AgentID_);
    obstacles_.update();

    /// 将融合后的信息发布到上层控制节点
    publish();
    /// 将自身的通过视觉检测到的原始障碍物信息，传输给队友，进行障碍物融合
    /// 这里obs_info是原始信息,obs_fuse是融合后的信息,这里之所以把融合后的信息也传出去是当时为Coach打下的补丁,希望能在Coach的UI上显示融合后的障碍物,其他机器人并未使用
    std::vector< DPoint > fuse_obs;
    obstacles_.getFuseObsTracker(fuse_obs);
    for(int i = 0 ; i < MAX_OBSNUMBER_CONST ; i++)
    {
        if(i < omni_obs.size())
            teammatesinfo_[AgentID_-1].obs_info_[i] = omni_obs[i];
        else
            teammatesinfo_[AgentID_-1].obs_info_[i].clear();

        if(i < fuse_obs.size() && !(!fuse_obs[i].x_ && !fuse_obs[i].y_))
            teammatesinfo_[AgentID_-1].obs_fuse_[i] = fuse_obs[i];
        else
            teammatesinfo_[AgentID_-1].obs_fuse_[i] = DPoint2s(-10000,-10000);
    }
    if(streaming_cout == 0)
    {
        /// 通过DB_put将RTDB中对应的信息更新
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
/// \brief 发布世界模型信息到机器人上层控制节点
/// 自身机器人信息、障碍物信息、COACH信息以及队友信息(主要是定位信息、足球信息、策略信息)
void
nubot::World_Model::publish()
{
    /// 将自身感知到的信息以及通信得到的队友信息，发送到上层控制节点
    /// 自身的感知信息在全向视觉的回调函数中已经放入teammatesinfo中,所以这里所有机器人的信息都来自teammatesinfo
    world_model_info_.robotinfo.clear();
    world_model_info_.robotinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        Robot & robot_info = teammatesinfo_[i].robot_info_;
        world_model_info_.robotinfo[i].AgentID = robot_info.getID();

        /// 静态传接球的分配信息
        world_model_info_.robotinfo[i].targetNum1=robot_info.getTargetNum(1);
        world_model_info_.robotinfo[i].targetNum2=robot_info.getTargetNum(2);
        world_model_info_.robotinfo[i].targetNum3=robot_info.getTargetNum(3);
        world_model_info_.robotinfo[i].targetNum4=robot_info.getTargetNum(4);
        world_model_info_.robotinfo[i].staticpassNum=robot_info.getpassNum();
        world_model_info_.robotinfo[i].staticcatchNum=robot_info.getcatchNum();
        /// 机器人的各种状态信息
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
    /// 自身感知到的障碍物，主要用于自身动作(抓球,避障...)
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
    /// 多机器人障碍物融合结果，主要用于协同配合(进攻,防守跑位)
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

    /// 发布球的信息,也分为两种,自身融合后的球,其他机器人看到的球
    world_model_info_.ballinfo.clear();
    world_model_info_.ballinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        BallObject ball_info;
        if(i == AgentID_-1)
            ball_info=ball_info_.fuse_ball_;
        else
            ball_info=teammatesinfo_[i].ball_info_;
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

    /// 将RTDB中的coach信息发布到上层节点
    world_model_info_.coachinfo.MatchMode =coach2robot_.MatchMode;
    world_model_info_.coachinfo.MatchType =coach2robot_.MatchType;
    world_model_info_.coachinfo.TestMode=coach2robot_.TestMode;
    world_model_info_.coachinfo.pointA.x=coach2robot_.pointA.x_;
    world_model_info_.coachinfo.pointA.y=coach2robot_.pointA.y_;
    world_model_info_.coachinfo.pointB.x=coach2robot_.pointB.x_;
    world_model_info_.coachinfo.pointB.y=coach2robot_.pointB.y_;
    world_model_info_.coachinfo.angleA=coach2robot_.angleA;
    world_model_info_.coachinfo.angleB=coach2robot_.angleB;
    world_model_info_.coachinfo.idA=coach2robot_.id_A;
    world_model_info_.coachinfo.idB=coach2robot_.id_B;
    world_model_info_.coachinfo.kickforce=coach2robot_.kick_force;

    ///将RTDB中的传接球信息发布到上层节点,pass_cmd(待修改)
    world_model_info_.pass_cmd.catch_id = -1;
    world_model_info_.pass_cmd.pass_id = -1;
    world_model_info_.pass_cmd.is_dynamic_pass  = false;
    world_model_info_.pass_cmd.is_static_pass  = false;
    world_model_info_.pass_cmd.is_passout = false;
    world_model_info_.pass_cmd.is_valid = false;

    for(int i = 0 ; i < OUR_TEAM; i++)
    {
        Teammatesinfo & teammates = teammatesinfo_[i];
        /// 不更新自己的传接球信息
        if(i != AgentID_-1)
        {
            if(teammates.pass_cmds_.isvalid && (teammates.pass_cmds_.is_static_pass||teammates.pass_cmds_.is_dynamic_pass||teammates.pass_cmds_.is_passout))
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

    /// 最后发布world_model_info_
    worldmodelinfo_pub_.publish(world_model_info_);
}

/// \brief 从RTDB中获得队友信息供自己融合并使用,在update中被调用
/// 在仿真环境下不使用
void
nubot::World_Model::updateInfo()
{
    /** 更新所有队友的信息，判断这些信息是否有效，根据时间戳*/
    double min_distance_ball = 100000000;
    teammateIDforBallSelected = -1;
    DPoint ball_vec = DPoint(0,0);
    bool   ball_vec_known = false;
    /// 利用DB_GET函数获得除自己以外的队友信息
    for( int i = 0 ; i < OUR_TEAM ; i++ )
    {
        if( AgentID_ != i+1 )
        {
            int ltime = DB_get(i+1, TEAMMATESINFO, &teammatesinfo_[i]);
            /// 接收到的队友信息，并记录其间隔时间，可能表示信息无效；
            teammatesinfo_[i].robot_info_.setlifetime(ltime);
            teammatesinfo_[i].ball_info_.setlifetime(ltime);
            teammatesinfo_[i].robot_info_.update();

            /// 通信没有中断（ltime）并且在获得足球信息的机器人上topic没有延迟太长时间（isValid()）
            if(ltime > 0  && ltime < NOT_DATAUPDATE && teammatesinfo_[i].ball_info_.isValid() && teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].ball_info_.setValid(true);
            else
                teammatesinfo_[i].ball_info_.setValid(false);

            if( teammatesinfo_[i].ball_info_.isValid() &&  teammatesinfo_[i].ball_info_.isLocationKnown())
            {
                /// 因为机器人检测到的球位置存在误差,所以不尽相同,选出其中距离最短的作为实际的球
                if(!ball_vec_known && teammatesinfo_[i].ball_info_.isVelocityKnown())
                {
                    ball_vec_known = true;
                    ball_vec = teammatesinfo_[i].ball_info_.getVelocity();
                }
                if(min_distance_ball > teammatesinfo_[i].ball_info_.getRealLocation().radius_)
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
            /// 判断接收到的传接球信息是否有效(待修改)
            if(ltime > 0 && ltime < NOT_DATAUPDATE && teammatesinfo_[i].pass_cmds_.isvalid && teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].pass_cmds_.isvalid = true;
            else
                teammatesinfo_[i].pass_cmds_.isvalid = false;

            /// 判断接收到的障碍物信息是否有效,并写入
            std::vector<ObstacleObject> obs_info;
            if(ltime > 0 && ltime < NOT_DATAUPDATE && teammatesinfo_[i].robot_info_.isValid())
            {
                for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                {
                    /// PolarLocation的初始赋值为10000,若此时仍为10000,则表明此障碍物无效
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

    /// 根据接收到的topic时间判断信息有效性
    ros::Time nowtime =  ros::Time::now();
    /// 更新全向视觉节点接收topic时间到当前的时间间隔，足球信息
    ros::Duration duration = nowtime - omni_update_time_;
    double self_time = duration.toNSec()/1000000.0;
    teammatesinfo_[AgentID_-1].robot_info_.setlifetime(self_time);
    teammatesinfo_[AgentID_-1].robot_info_.update();
    if(self_time < 0 || self_time > NOT_DATAUPDATE)
        obstacles_.clearOmniObstacles(AgentID_);
    ball_info_.sensor_ball_[OMNI_BALL].setlifetime(self_time);

    /// 更新深度相机节点接收topic时间到当前的时间间隔，足球信息
    duration = nowtime -kinect_update_time_;
    self_time = duration.toNSec()/1000000.0;
    if(self_time < 0 || self_time > NOT_DATAUPDATE)
        obstacles_.clearKinectObstacles(AgentID_);
    ball_info_.sensor_ball_[KINECT_BALL].setlifetime(self_time);
}
