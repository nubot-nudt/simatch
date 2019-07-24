#ifndef _NUBOT_WORLD_MODEL_INFO_H
#define _NUBOT_WORLD_MODEL_INFO_H

#include "core.hpp"
#include "world_model/robot.h"
#include "world_model/ball.h"
#include "world_model/teammatesinfo.h"
#include "nubot/nubot_control/dribblestate.hpp"
#include "nubot/nubot_control/fieldinformation.h"
#include "ros/ros.h"
#define CENTRALIZE_POSITION
const double ConstDribbleDisFirst=50;
namespace nubot
{
const int PASSSTATE_TIME_DELAY = 100;
enum{ KickBySelf = 0, KickByOpp =1, NoKick=2};
const double KickBySelfTime = 10;
class PassState
{
public:
    bool is_passing_;
    int  time_lock_;  // 1s/30ms=30
    int  pass_id_;
    int  catch_id_;
    DPoint pass_pt_;
    DPoint catch_pt_;
    bool is_dynamic_pass_;
    bool is_static_pass_;
    PassState():is_passing_(false){
        reset();
    }
    void set(const int & pass_id, const int & catch_id,
             const DPoint & pass_pt, const DPoint & catch_pt,
             const bool & _dynamic_pass,const bool &_static_pass)
    {
        is_passing_   = true;
        is_dynamic_pass_ = _dynamic_pass;
        is_static_pass_  = _static_pass;
        pass_id_      = pass_id;
        catch_id_     = catch_id;
        pass_pt_      = pass_pt;
        catch_pt_     = catch_pt;
        time_lock_    = PASSSTATE_TIME_DELAY;
    }
    void
    reset(){
        is_passing_      = false;
        is_dynamic_pass_ = false;
        is_static_pass_  = false;
        pass_id_ = -1;
        catch_id_ =-1;
        time_lock_ = 0;
    }
    void update()//ensure this update() is called in every callback
    {
        if(time_lock_ > 0 )
            time_lock_--;
        else
            reset();
    }
};

class World_Model_Info
{
public:
    std::vector<nubot::Robot>      RobotInfo_; //! 机器人信息
    std::vector<nubot::BallObject> BallInfo_;  //!
    std::vector<nubot::DPoint>     Obstacles_; //!
    std::vector<nubot::DPoint>     Opponents_; //!
    PassCommands      pass_cmds_; // 接收到的队友机器人发球的传球命令
    MessageFromCoach  CoachInfo_; // 接收到的COACH信息
    int BallInfoState_;           // 当前看到足球的状态自己、队友或者没有看到足球
    DPoint lastBallPosition_;     // 上一帧足球所在的位置
    int  NoSeeBallNums_;          // 连续几帧没有看到足球
    int  AgentID_;                // 机器人自身的ID
    int  CurActiveRobotNums_;     // 当前活跃的机器人数目
    int  PreActiveRobotNums_;     // 上一帧活跃的机器人数目
    int  CurActiveNotGoalieNums_; // 当前除去守门员活跃的机器人书目
    int  PreActiveNotGoalieNums_; // 上一帧除去守门员活跃的机器人书目
    bool RegainBallInOurFiled_;   // 得到球是在自己的半场上，现在必须传球,false可以直接射门；
    DribbleState DribbleState_;   // 判断带球的状态；
    bool isShootflg;              // 是否踢球
    FieldInformation field_info_;
    bool IsMoveForStartCommand_;
    bool IsOurDribble_;
    /**  传球有关的状态*/
    PassState pass_state_;        //接球或者是踢球，主要用于锁定状态，接球通过通信锁定
    DPoint catch_pt_;            /** 接球机器人占位*/
    DPoint pass_pt_;             /** 传球机器人位置*/
    int    catch_ID_;            /** 接球机器人ID*/
    int    pass_ID_;             /** 传球机器人的ID*/
    double kick_force_;          /** 踢球的力量*/
    bool   is_dynamic_pass_;     /** 准备动态传球*/
    bool   is_static_pass_;      /** 准备静态传球*/
    bool   is_passed_out_;       /** 足球已经踢出*/
    int    KickSelection_;        /** 对方发球还是我方发球*/
    double pass_direction_;
    bool   can_pass_ ;
    std::vector<double> angle_;
    std::vector<double> angle_dis_;
    DPoint assist_pt_;
    double pass_sight_;
    DPoint middle_pt_;
    DPoint passive_pt_;
    double static_pass_time_;
    double static_ball_dis_;
public:
    World_Model_Info()
    {
        RobotInfo_.resize(OUR_TEAM);
        BallInfo_.resize(OUR_TEAM);
        angle_.reserve(30);
        angle_dis_.reserve(30);
        AgentID_ = 0;
        CurActiveRobotNums_ = 0;
        PreActiveRobotNums_ = 0;
        PreActiveNotGoalieNums_ = 0;
        CurActiveNotGoalieNums_ = 0;
        BallInfoState_ = NOTSEEBALL;
        RegainBallInOurFiled_ = false;
        Obstacles_.reserve(25);
        isShootflg=false;
//        CoachInfo_.Head = AgentID_;
//        CoachInfo_.CoachStrategy = STRATEGY_DEFEND;
        CoachInfo_.MatchMode = STOPROBOT;
        CoachInfo_.MatchType = STOPROBOT;
        IsMoveForStartCommand_ =false;
        lastBallPosition_ = DPoint(-650,0);
        NoSeeBallNums_ = 0;
        KickSelection_ = NoKick;
        pass_direction_ = 0;
        assist_pt_  = DPoint(0,0);
        passive_pt_ = DPoint(0,0);
        pass_sight_ = 0;
        static_pass_time_ = 0;
        static_ball_dis_  = 0;
        clearPassState(true);
        pass_state_.reset();
        can_pass_ = false;
    }
    /** 计算场地上活跃的机器人数目*/
    void caculateActiveRobots()
    {
        PreActiveRobotNums_     = CurActiveRobotNums_;
        PreActiveNotGoalieNums_ = CurActiveNotGoalieNums_;
        CurActiveNotGoalieNums_ = CurActiveRobotNums_ = 0;
        for(std::size_t i = 1 ; i < OUR_TEAM ; i++)
            CurActiveNotGoalieNums_ += RobotInfo_[i].isValid();
        CurActiveRobotNums_ = CurActiveNotGoalieNums_ + RobotInfo_[0].isValid();
    }

    /** @param ball_holding为从nubot_hwcontroller返回的服务状态
    *  函数的功能给是判断机器人是否带上足球
    */
    void checkDribble(const bool & ball_holding)
    {
        bool pre_dribble   = DribbleState_.is_dribble_;
        DPoint robot_pos = RobotInfo_[AgentID_-1].getLocation();
        DPoint ball_pos  = BallInfo_[AgentID_-1].getGlobalLocation();

        double dis2b = robot_pos.distance(ball_pos);
        bool dribblecheck = dis2b <= ConstDribbleDisFirst ? ball_holding : false;

      //  ROS_INFO("%d %d ", AgentID_,dribblecheck );
        DribbleState_.update(dribblecheck,robot_pos,ball_pos);
        /** 在自身半场得到的足球，必须经过传球得分才能有效*/
        if(DribbleState_.is_dribble_ && !pre_dribble)
        {
            if(ball_pos.x_ < 0)
              RegainBallInOurFiled_ = true;
        }
        static int DribbleCount = 0;
        if(DribbleState_.is_dribble_)
            DribbleCount = 0;
        if(!DribbleState_.is_dribble_)  DribbleCount++;
        if(DribbleCount > 10)           RegainBallInOurFiled_ = false;
        //更新dribble_状态；
        RobotInfo_[AgentID_-1].setDribbleState(DribbleState_.is_dribble_);
       // ROS_INFO("%d %.f %.f %.f %d %d",AgentID_,dis2b,ConstDribbleDisFirst,ConstDribbleDisSecond,dribblecheck,DribbleState_.is_dribble_);
        IsOurDribble_ = false; //是否我方带球；
        for(int i = 0; i <OUR_TEAM;i++)
        {
            if(RobotInfo_[i].isValid() && RobotInfo_[i].getDribbleState())
                IsOurDribble_ =true;
        }
    }

    //! 检测机器人是否能够移动，即发球时候的状态
    void isMoveForStartCommand()
    {
        static bool is_start = false;
        //! 表示是否重新点击开始命令,
        if(CoachInfo_.MatchMode!=STARTROBOT)
            is_start = false;
        //! STOPROBOT命令之后不能移动。
        if(CoachInfo_.MatchMode==STOPROBOT)
            IsMoveForStartCommand_ = false;

        static ros::Time start_time = ros::Time::now();
        static DPoint ball_pos = BallInfo_[AgentID_-1].getGlobalLocation();
        if(CoachInfo_.MatchMode==STARTROBOT)
        {
            if(CoachInfo_.MatchType==OUR_KICKOFF    || CoachInfo_.MatchType==OUR_THROWIN||
                    CoachInfo_.MatchType==OUR_PENALTY    || CoachInfo_.MatchType==OUR_GOALKICK||
                    CoachInfo_.MatchType==OUR_CORNERKICK || CoachInfo_.MatchType==OUR_FREEKICK||
                    CoachInfo_.MatchType==STOPROBOT      ||CoachInfo_.MatchType==DROPBALL)
                IsMoveForStartCommand_  = true;
            else if(CoachInfo_.MatchType==OPP_KICKOFF    || CoachInfo_.MatchType==OPP_THROWIN||
                    CoachInfo_.MatchType==OPP_PENALTY    || CoachInfo_.MatchType==OPP_GOALKICK||
                    CoachInfo_.MatchType==OPP_CORNERKICK || CoachInfo_.MatchType==OPP_FREEKICK)
            {
                if(!is_start)
                {
                    is_start=true;
                    start_time = ros::Time::now();
                    ball_pos =  BallInfo_[AgentID_-1].getGlobalLocation();
                }
                ros::Duration duration = ros::Time::now() - start_time;
                if(duration.toSec()>7)                        //dw 4.16  等待时间调整到7s
                    IsMoveForStartCommand_  = true;
                else if(BallInfoState_!=NOTSEEBALL)
                {
                    if(BallInfo_[AgentID_-1].getGlobalLocation().distance(ball_pos) > 75)
                        IsMoveForStartCommand_  = true;
                }
            }
        }
    }

    void clearPassState(bool isclearId)
    {
        if(isclearId)
        {
          catch_ID_= -1;
          pass_ID_ = -1;
         }
        KickSelection_ = NoKick;
        is_static_pass_  = false;
        is_dynamic_pass_ = false;
        is_passed_out_   = false;
    }
    void caculatePassPosition()
    {
        DPoint robot_pos = RobotInfo_[AgentID_-1].getLocation();
        DPoint ball_pos  = BallInfo_[AgentID_-1].getGlobalLocation();
        DPoint pass_point_temp;//候选传球目标点
        //↓↓↓↓↓传球点的计算↓↓↓↓↓
        //1.传球方向角度的阈值计算
        double direction_low;//传球方向下限
        double direction_high;//传球方向上限
        bool   thresh_cross_pi = false;
        static bool upper_loc = robot_pos.y_ > 0; false, true; //
        if(ball_pos.y_ >200 * WIDTH_RATIO)
            upper_loc = true;
        else if(ball_pos.y_ <-200 * WIDTH_RATIO)
            upper_loc = false;
        if(ball_pos.x_ < 0 && upper_loc) //
        {
            if(ball_pos.y_ > 200)
            {
                direction_low =   DPoint(200.0, -200*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(550.0,  200*WIDTH_RATIO).angle(ball_pos).radian_;
            }
            else if( ball_pos.y_ > 0 &&  ball_pos.y_ < 200)
            {
                direction_low   =  DPoint(200.0, -200*WIDTH_RATIO-200*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(550.0,  200*WIDTH_RATIO-200*WIDTH_RATIO).angle(ball_pos).radian_;
            }
            else
            {
                direction_low   =   DPoint(200.0, -200*WIDTH_RATIO-400*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(550.0,  200*WIDTH_RATIO-400*WIDTH_RATIO).angle(ball_pos).radian_;
            }
        }
        else if(ball_pos.x_ < 0 && !upper_loc)
        {
            if(ball_pos.y_ <-200)
            {
                direction_low   =  DPoint(550.0, -200*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(200.0,  200*WIDTH_RATIO).angle(ball_pos).radian_;
            }
            else if( ball_pos.y_ < 0 &&  ball_pos.y_ > -200)
            {
                direction_low  =   DPoint(550.0 , -200*WIDTH_RATIO+200*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(200.0,  200*WIDTH_RATIO+200*WIDTH_RATIO).angle(ball_pos).radian_;
            }
            else
            {
                direction_low   =   DPoint(550.0, -200*WIDTH_RATIO+400*WIDTH_RATIO).angle(ball_pos).radian_;
                direction_high  =  DPoint(200.0,  200*WIDTH_RATIO+400*WIDTH_RATIO).angle(ball_pos).radian_;
            }

        }
        else//对方半场传球点搜索范围
        {
            if(ball_pos.x_>450)//球比较深入，可以往回传(阈值跨越正负PI)
            {
                thresh_cross_pi = true;
                if(ball_pos.y_ > 0)
                {
                    direction_low  =  DPoint(200.0, 500*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(200.0,-500*WIDTH_RATIO).angle(ball_pos).radian_;
                }
                else
                {
                    direction_low  =  DPoint(200.0, 500*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(200.0,-500*WIDTH_RATIO).angle(ball_pos).radian_;
                }
            }
            else
            {
                if(ball_pos.y_>150*WIDTH_RATIO)//向右传
                {
                    direction_low  =  DPoint(200.0,  -500*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(600.0, -500*WIDTH_RATIO).angle(ball_pos).radian_;
                }
                else if(ball_pos.y_<-150*WIDTH_RATIO)//向左传
                {
                    direction_low  =  DPoint(600.0, 600*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(200.0,   600*WIDTH_RATIO).angle(ball_pos).radian_;
                }
                else if(pass_direction_<0)//向右传
                {
                    direction_low  =  DPoint(200.0,  -600*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(600.0, -600*WIDTH_RATIO).angle(ball_pos).radian_;
                }
                else//向左传
                {
                    direction_low  =  DPoint(600.0, 600*WIDTH_RATIO).angle(ball_pos).radian_;
                    direction_high =  DPoint(200.0, 600*WIDTH_RATIO).angle(ball_pos).radian_;
                }
            }
        }
        double pass_direction_temp;
        double sight_temp = 0;
        //2.首先检测当前场上状态是否还能继续上次的传球站位点,要求上次传球方向也在这次传球方向阈值内
        if( can_pass_ && (
                    (!thresh_cross_pi && pass_direction_>=direction_low && pass_direction_<=direction_high )
                    ||(thresh_cross_pi && pass_direction_>=-SINGLEPI_CONSTANT && pass_direction_<=direction_high)
                    ||(thresh_cross_pi && pass_direction_>=direction_low && pass_direction_<=SINGLEPI_CONSTANT)))//上次传球状态有效,则检测现在是否能继续使用
        {
            double direction_low_last  = pass_direction_-SINGLEPI_CONSTANT/6.0;//将上次传球方向微微放宽，同样要限制放宽后角度，用于判断是否可以沿用上次传球点
            double direction_high_last = pass_direction_+SINGLEPI_CONSTANT/6.0;//将上次传球方向微微放宽，同样要限制放宽后角度，用于判断是否可以沿用上次传球点
            if(!thresh_cross_pi)
            {
                if(direction_low_last<direction_low)   direction_low_last = direction_low;
                if(direction_high_last>direction_high) direction_high_last = direction_high;
            }
            else
            {
                if(direction_low_last<-SINGLEPI_CONSTANT) direction_low_last  += DOUBLEPI_CONSTANT;
                if(direction_high_last>SINGLEPI_CONSTANT) direction_high_last -= DOUBLEPI_CONSTANT;
                if(direction_low_last<direction_low && direction_low_last>0)    direction_low_last = direction_low;
                if(direction_high_last>direction_high && direction_high_last<0) direction_high_last = direction_high;
            }
            if(ball_pos.x_<0)
                sight_temp = searchMaxSight( pass_direction_temp, ball_pos,  direction_low_last, direction_high_last, 800 );
            else
                sight_temp = searchMaxSight( pass_direction_temp, ball_pos,  direction_low_last, direction_high_last, 400 );
            if( (!thresh_cross_pi && sight_temp==direction_high_last-direction_low_last)
                    ||(thresh_cross_pi && sight_temp==direction_high_last-direction_low_last+DOUBLEPI_CONSTANT))//当direction_low_last与direction_high_last之间没有障碍时，就不适用“最优”传球点了，而是维持上次传球点
                pass_direction_temp = pass_direction_;
        }
        if( sight_temp>SINGLEPI_CONSTANT/6 ) //上次的传球站位点还能继续使用，当然有微小偏移
        {
            if(ball_pos.x_<0)
            {
                pass_point_temp = ball_pos + DPoint(PPoint(Angle(pass_direction_temp),ball_pos.length()+300));
                if( pass_point_temp.x_ < 200 )      pass_point_temp.x_ = 200;
                else if( pass_point_temp.x_ > 400 ) pass_point_temp.x_ = 400;
            }
            else
            {
                pass_point_temp = ball_pos + DPoint(PPoint(Angle(pass_direction_temp),400));
                if( pass_point_temp.x_ < 100 )      pass_point_temp.x_ = 100;
                else if( pass_point_temp.x_ > 500 ) pass_point_temp.x_ = 500;
            }
            if( pass_point_temp.y_<-500*WIDTH_RATIO )     pass_point_temp.y_ = -500*WIDTH_RATIO;
            else if( pass_point_temp.y_>500*WIDTH_RATIO ) pass_point_temp.y_ = 500*WIDTH_RATIO;
        }
        else//上次的传球站位点不能继续使用,重新搜索新的传球点
        {
            sight_temp = searchMaxSight( pass_direction_temp, ball_pos, direction_low, direction_high, 400 );
            if(ball_pos.x_<0)
            {
                pass_point_temp = ball_pos + DPoint(PPoint(Angle(pass_direction_temp),ball_pos.length()+300));
                if( pass_point_temp.x_ < 200 ) pass_point_temp.x_ = 200;//防止接球点离中线太近
                else if( pass_point_temp.x_ > 400 ) pass_point_temp.x_ = 400;//防止接球点离中线太远
            }
            else
            {
                pass_point_temp = ball_pos + DPoint(PPoint(Angle(pass_direction_temp),350));
                if( pass_point_temp.x_ < 100 )      pass_point_temp.x_ = 100;//防止接球点离中线太近
                else if( pass_point_temp.x_ > 500 ) pass_point_temp.x_ = 500;//防止接球点离中线太远
            }
            if( pass_point_temp.y_<-500*WIDTH_RATIO )     pass_point_temp.y_ = -500*WIDTH_RATIO;   //防止接球点离边界太近
            else if( pass_point_temp.y_>500*WIDTH_RATIO ) pass_point_temp.y_ = 500*WIDTH_RATIO;//防止接球点离边界太近
        }
        //综上求得的传球点传球视野足够，可以传球
        if(sight_temp>SINGLEPI_CONSTANT/6)
        {
            can_pass_ = true;
            assist_pt_ = pass_point_temp;
            pass_sight_ = sight_temp;
            pass_direction_ = pass_direction_temp;
        }
        else
            can_pass_ = false;
        /*DPoint ball_direction = robot_pos -ball_pos;//从球门到球的方向向量
        double distance = ball_direction.length();
        if(distance!=0 && distance < 350 )
           assist_pt_ = ball_pos + 350.0/distance * ball_direction;*/
        //辅助防守盯人，防止对方传球
        if(ball_pos.x_ < 0)
        {
            //主防守站在球与球门连线上
            DPoint ball_direction = ball_pos-DPoint(-900,0);//从球门到球的方向向量
            double distance = ball_direction.length();
            if(ball_direction.x_==0 && ball_direction.y_==0)
                distance +=1;
            ball_direction = 1.0/distance * ball_direction;
            DPoint def_point = DPoint(-900,0)+ball_direction*350;
            passive_pt_ = def_point;
            if(ball_pos.distance(def_point)<100)//防止防守队员和主攻都顶着球,要将防守队员错开点
            {
                if(CurActiveNotGoalieNums_ == 4  )//有两个后场防守(除主攻外)
                {
                    if( ball_pos.y_>0 && ball_pos.y_<300*WIDTH_RATIO ) passive_pt_.y_ += 200*WIDTH_RATIO;
                    else if( ball_pos.y_<-300*WIDTH_RATIO )            passive_pt_.y_ += 200*WIDTH_RATIO;
                    else                                               passive_pt_.y_ -= 200*WIDTH_RATIO;
                }
                else//只有一个后场防守(除主攻外)
                {
                    if( ball_pos.y_<0 )           passive_pt_.y_ += 200*WIDTH_RATIO;
                    else                          passive_pt_.y_ -= 200*WIDTH_RATIO;
                }
                if(fabs(passive_pt_.y_)<375*WIDTH_RATIO && passive_pt_.x_<-625)
                    passive_pt_.x_ = -625;//防止进入禁区，因为假设主攻已经进去了
            }

            //防止机器人中场占位发生突变，首先保持上一帧的
            middle_pt_.x_ = ball_pos.x_-100;
            if(robot_pos.y_ > 0)
                middle_pt_.y_ = 400*WIDTH_RATIO;
            else
                middle_pt_.y_ = -400*WIDTH_RATIO;
            if(ball_pos.y_<-150)
                middle_pt_.y_ = 400*WIDTH_RATIO;
            if(ball_pos.y_>150)
                middle_pt_.y_ = -400*WIDTH_RATIO;

            for(int i=0; i< Opponents_.size(); i++)
            {
                DPoint &obs = Opponents_[i];
                if( obs.x_>-700 && obs.distance(middle_pt_)<250*WIDTH_RATIO )//有需要防守的对手
                {
                    middle_pt_ = obs+100.0/(obs.distance(ball_pos)+1)*(ball_pos-obs);
                    break;
                }
            }
            if(fabs(middle_pt_.y_)>500*WIDTH_RATIO )//防止出界
            {
                if(ball_pos.y_<0)
                    middle_pt_.y_ = 500*WIDTH_RATIO;
                else
                    middle_pt_.y_ = -500*WIDTH_RATIO;
            }
            if(middle_pt_.x_<-625) middle_pt_.x_ = -625;//防止退的太后
        }
        else
        {
            passive_pt_ = DPoint(-200,0);
            for(int i=0; i< Opponents_.size(); i++)
            {
                DPoint &obs = Opponents_[i];
                //我方半场有需要防守的对手
                if( obs.x_<-150 && obs.x_>-650 && obs.y_<500*WIDTH_RATIO && obs.y_>-500*WIDTH_RATIO )
                {
                    passive_pt_ = obs+100.0/(obs.distance(ball_pos)+1)*(ball_pos-obs); //
                    break;
                }
            }
            DPoint assist_pt = assist_pt_;
            for(int i = i ; i < OUR_TEAM;i++)
            {
                if(RobotInfo_[i].isValid() && RobotInfo_[i].getCurrentRole() == ASSISTANT && i != AgentID_-1)
                     assist_pt =RobotInfo_[i].getLocation();
            }
            middle_pt_.x_ = 100;
            if(robot_pos.y_ > 0)
                middle_pt_.y_ = 400*WIDTH_RATIO;
            else
                middle_pt_.y_ = -400*WIDTH_RATIO;
            if(assist_pt.y_<-150)
                middle_pt_.y_ = 400*WIDTH_RATIO;
            if(assist_pt.y_>150)
                middle_pt_.y_ = -400*WIDTH_RATIO;
            //! 自身为助攻时候，中场位助攻的位置
        }
        if(RobotInfo_[AgentID_-1].isValid() && RobotInfo_[AgentID_-1].getCurrentRole() == ASSISTANT)
            middle_pt_ = assist_pt_;
    }
    double
    searchMaxSight(double &dest_angle, const DPoint &_point_from, const double &_angle_min, const double &_angle_max, const double &_max_distance)
    {
    //功能：根据与_point_from距离_max_distance以内的障碍物信息，搜索一个视野最佳的方向，可用于传球配合等功能
    //实现：先计算所有_max_distance范围内障碍物的角度信息，并按从小到大顺序存储；接着计算两两之间角度差，找到其角分线在角度阈值范围内的，最大的角度差；
    //      由于以上计算会漏掉哪些角分线不在角度阈值内，但是角分线稍微偏一点就可以在阈值内的情况，所以最后需要将角度阈值的两条边界看做候选最佳方向，与前述结果比较

        bool is_cross_PI = false;//给定的角度阈值是否跨越±PI，阈值判断时用到
        if( _angle_min > _angle_max )
            is_cross_PI = true;
        double best_angle=0;//最大角度间隔对应的中分角
        double max_angle_dis = 0;//最大角度间隔
        double obs_num = Opponents_.size();// 障碍物的数目；

        angle_dis_.clear();
        angle_.clear();
        angle_dis_.resize(obs_num);
        angle_.resize(obs_num);

        if( obs_num == 0 )//场地内没有障碍物
        {
            if(!is_cross_PI)
            {
                best_angle = (_angle_min + _angle_max ) / 2;
                max_angle_dis = _angle_max -_angle_min;
            }
            else
            {
                best_angle = (_angle_min + _angle_max + DOUBLEPI_CONSTANT ) / 2;
                if( best_angle>SINGLEPI_CONSTANT ) best_angle -= DOUBLEPI_CONSTANT;
                max_angle_dis = _angle_max - _angle_min + DOUBLEPI_CONSTANT;
            }
            dest_angle = best_angle;
            return max_angle_dis;
        }

        int num = 0;//有效角度的个数（即在_max_distance阈值范围之内的）
        for(int i=0; i<obs_num; i++)
            angle_[i] = -SINGLEPI_CONSTANT;
        double angle_current;
        for(int i=0; i<obs_num; i++)
        {
            if( (Opponents_[i]-_point_from).length()>_max_distance )
                continue;
            angle_current = Opponents_[i].angle(_point_from).radian_;
            if(num==0)
                angle_[0] = angle_current;
            else
            for(int j=num; j>=0; j--)//angle 从小到大排列存储
            {
                if(j==0)
                    angle_[j] = angle_current;
                else if(angle_current>angle_[j-1])
                {
                    angle_[j] = angle_current;
                    break;
                }
                else
                    angle_[j] = angle_[j-1];
            }
            num++;
        }


        //判断某个角度是否在阈值范围内的宏定义
  #define IsInThresh(in) ( !is_cross_PI && in > _angle_min && in < _angle_max ) \
        || ( is_cross_PI && in > -SINGLEPI_CONSTANT        && in < _angle_max ) \
        || ( is_cross_PI && in > _angle_min && in < SINGLEPI_CONSTANT         )

        //检测角度阈值内是否有障碍物
        bool is_obs_valid = false;
        for( int i=0; i<num; i++ )
        {
            if( IsInThresh(angle_[i]) )
                is_obs_valid = true;
        }
        if( !is_obs_valid )
        {
            if(!is_cross_PI)
            {
                best_angle = (_angle_min + _angle_max ) / 2;
                max_angle_dis = _angle_max -_angle_min;
            }
            else
            {
                best_angle = (_angle_min + _angle_max + DOUBLEPI_CONSTANT ) / 2;
                if( best_angle>SINGLEPI_CONSTANT ) best_angle -= DOUBLEPI_CONSTANT;
                max_angle_dis = _angle_max - _angle_min + DOUBLEPI_CONSTANT;
            }
            dest_angle = best_angle;
            return max_angle_dis;
        }

        //有障碍物，则需要计算相邻障碍物之间间隔，并搜索最大间隔↓↓↓
        //第一个障碍物和最后一个障碍物间隔的计算不方便放在for循环中，单独计算（只有一个障碍物时也兼容）
        angle_dis_[0] = angle_[0] - angle_[num-1] + DOUBLEPI_CONSTANT;
        double best_angle_temp = (angle_[0] + angle_[num-1] + DOUBLEPI_CONSTANT)/2;
        if( best_angle_temp>SINGLEPI_CONSTANT ) best_angle_temp -= DOUBLEPI_CONSTANT;
        if( IsInThresh(best_angle_temp) )
        {
            max_angle_dis = angle_dis_[0];
            best_angle = best_angle_temp;
        }
        for(int i=1; i<num; i++)
        {
            angle_dis_[i] = angle_[i] -angle_[i-1];
            if( angle_dis_[i]>max_angle_dis )
            {
                best_angle_temp = (angle_[i] + angle_[i-1])/2;
                if( IsInThresh(best_angle_temp) )
                {
                    max_angle_dis = angle_dis_[i];
                    best_angle = best_angle_temp;
                }
            }
        }

        //最后计算两条角度阈值边界的视野
        double angle_dis_min = 0;
        if( _angle_min<angle_[0] )
            angle_dis_min = std::min( _angle_min-angle_[num-1]+DOUBLEPI_CONSTANT, angle_[0]-_angle_min);
        else if( _angle_min>angle_[num-1] )
            angle_dis_min = std::min( _angle_min-angle_[num-1], angle_[0]-_angle_min+DOUBLEPI_CONSTANT);
        else for(int i=1; i<num; i++)
        {
            if(_angle_min<angle_[i])
            {
                angle_dis_min = std::min( _angle_min-angle_[i-1], angle_[i]-_angle_min);
                break;
            }
        }

        double angle_dis_max = 0;
        if( _angle_max<angle_[0] )
            angle_dis_max = std::min( _angle_max-angle_[num-1]+DOUBLEPI_CONSTANT, angle_[0]-_angle_max);
        else if( _angle_max>angle_[num-1] )
            angle_dis_max = std::min( _angle_max-angle_[num-1], angle_[0]-_angle_max+DOUBLEPI_CONSTANT);
        else for(int i=1; i<num; i++)
        {
            if(_angle_max<angle_[i])
            {
                angle_dis_max = std::min( _angle_max-angle_[i-1], angle_[i]-_angle_max);
                break;
            }
        }
        if( angle_dis_min*2 > max_angle_dis )
        {
            max_angle_dis = angle_dis_min*2;
            best_angle = _angle_min;
        }
        if( angle_dis_max*2 > max_angle_dis )
        {
            max_angle_dis = angle_dis_max*2;
            best_angle = _angle_max;
        }

        dest_angle = best_angle;//在最后才去赋值仅仅是防止可能的多线程冲突而已,意义不大
        return max_angle_dis;
    #undef IsInThresh
    }

    void
    update(const bool & ball_holding)
    {
        //! 表示正在传球过程中，因此其他机器人必须根据该状态确定角色*/
        if(pass_cmds_.isvalid && pass_cmds_.is_passout)
        {
            pass_state_.set(pass_cmds_.passrobot_id,
                            pass_cmds_.catchrobot_id,
                            pass_cmds_.pass_pt,
                            pass_cmds_.catch_pt,
                            pass_cmds_.is_dynamic_pass,
                            pass_cmds_.is_static_pass);
            std::cout<<" pass_done_com: "<<std::endl;
            clearPassState(true); //球已经传出之后，所有的准备传球信息清空
        }
        pass_state_.update();
        if(BallInfoState_ != NOTSEEBALL)
        {
            lastBallPosition_ = BallInfo_[AgentID_-1].getGlobalLocation();
            NoSeeBallNums_ = 0;
        }
        else
            NoSeeBallNums_++;
        caculateActiveRobots();
        checkDribble(ball_holding);
#ifdef CENTRALIZE_POSITION
        caculatePassPosition();
#endif
        isMoveForStartCommand();
    }
};

}

#endif /** _NUBOT_WORLD_MODEL_INFO_H*/
