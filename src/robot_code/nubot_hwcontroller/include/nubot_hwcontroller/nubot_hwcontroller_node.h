#ifndef BALL_HANDLE_H
#define BALL_HANDLE_H

#include <stdio.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>
#include <vector>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <ros/ros.h>

#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>

#include "nubot_common/Teleop_joy.h"
#include "nubot_common/VelCmd.h"
#include "nubot_common/ActionCmd.h"
#include "nubot_common/OdoInfo.h"
#include "nubot_common/BallIsHolding.h"
#include "DPoint.hpp"
#include "core.hpp"
#include "dynamic_reconfigure/server.h"
//#include "nubot_hwcontroller/controllerConfig.h"
#include "nubot_hwcontroller/DebugInfo.h"

/// \brief 底层程序主要集中在这个cpp中,涉及到的函数和变量众多，编程风格也别具一格
/// 1.新加入后的部分变量没有在构造函数中初始化,存在风险
/// 2.将以前的service模式改成topic模式后,彻底解决机器人失能问题
/// 3.允许机器人程序开启后不标定带球距离,前提是带球机构足够稳定
/// 4.不要轻易修改soem中的程序

using namespace std;

class Nubot_HWController
{
public:
    Nubot_HWController(int argc,char** argv);
    ~Nubot_HWController();

    void  Timer1_Process(const ros::TimerEvent &event);

    void  SetAction(const nubot_common::ActionCmd::ConstPtr& cmd);
    void  BaseController(/*const ros::TimerEvent &event*/);
    void  calculateSpeed();
    float basicPDControl(float pgain,float dgain, float err,float err1, float maxval);
//    void  accelerateLimit(const bool &use_convected_acc=false);
//    nubot::DPoint select_accLimit();
    void  adjustPD(float pval, float dval, float weight=0.5);
    void  move2target();
    void  movewithball();
    void  rotate2AbsOrienation();

private:
//    boost::shared_ptr<ros::NodeHandle> n;
    ros::NodeHandle n;
    ros::Timer timer1;
    ros::Subscriber     actioncmd_sub_;
    ros::Publisher      motor_cmd_pub_;

public:
    ros::Time TimeFromHighLevel_;
    int   BallHandleEnable;
    bool  ShootEnable;
    float ShootPower;
    int   ShootPos;
    bool  RodPos;
    int   ShootState;
    int   shootcnt;
    bool  ShootDone;
    bool  PowerState,PowerState_Last;
    bool  RobotStuck;
    string name;
    int number;

    int motor_speed[2];
    int &motor_left,&motor_right;
    int motor_up;
    int Motor_Readings[4];
    int Real_v[6];
    double Vx,Vy,w,Vx_diff,w_diff;
    double Vx_cmd,Vy_cmd,w_cmd;
    int    RotationMode;
    int    acc_state,wacc_state;
    double Real_Vx,Real_Vy,Real_w;
    double FBRatioL,FBRatioR;
    float  FFRatio_Set;

    /// 带球的PID
    double P,I,D;

    /// 由上层节点传到底层的运动参数
    char move_action_;
    char rotate_action_;
    int  rotate_mode_;
    nubot::DPoint2s target_;
    nubot::DPoint2s target_vel_;
    float target_ori_;
    float maxvel_;
    float maxw_;
    nubot::DPoint2s robot_pos_;
    nubot::DPoint2s robot_vel_;
    nubot::Angle  robot_ori_;
    float robot_w_;
    float p_move_;
    float d_move_;
    float p_rotation_;
    float d_rotation_;

    /// 视觉里程计融合
    std::vector<double> rel_v;

    double LeverPos_Left,LeverPos_Right,LeverPos_Diff;
    double LeverPos_SetPoint;
    double BallSensor_LBias,BallSensor_RBias,BallSensor_LStroke,BallSensor_RStroke;
    bool   IsBallSensorInited;
    bool   BallSensor_IsHolding,IsBallLost;
    int    HoldingCnt, UnholdingCnt;
};


#endif
