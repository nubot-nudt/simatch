#ifndef _NUBOT_TEAMMATESINFO_H
#define _NUBOT_TEAMMATESINFO_H

#include "world_model/ball.h"
#include "world_model/robot.h"
#include "world_model/obstacles.h"

namespace nubot{

/// \brief 关于传接球信息的类(待修改)
class PassCommands
{
 public:
    PassCommands()
    {
        passrobot_id = -1;
        catchrobot_id = -1;
        is_dynamic_pass = false;
        is_static_pass  = false;
        is_passout =false;
        pass_pt = DPoint(900,0);
        catch_pt = DPoint(900,0);
        isvalid = false;
    }
public:
    int  passrobot_id;
    int  catchrobot_id;
    bool is_dynamic_pass;
    bool is_static_pass;
    bool is_passout;
    DPoint pass_pt;
    DPoint catch_pt;
    bool  isvalid;
};

/// \brief 来自coach的信息
/// 为了减少RTDB的通信量,在该结构体中的部分值存在复用,因此它的命名并不完全表征其含义
struct MessageFromCoach
{
    char MatchMode;
    char MatchType;
    char TestMode;
    DPoint2s pointA;
    DPoint2s pointB;
    short angleA;
    short angleB;
    char id_A;
    char id_B;
    char kick_force;
};

/// \brief 队友信息
/// 通过RTDB传输的信息,传输量大(500mb左右)
class Teammatesinfo
{
public:
    /// 该机器人信息
    Robot          robot_info_;
    /// 该机器人识别的足球信息
    BallObject     ball_info_;
    /// 该机器人识别到的原始的障碍物信息
    ObstacleObject obs_info_[MAX_OBSNUMBER_CONST];
    /// 传接球信息
    PassCommands   pass_cmds_;
    /// 该机器人识别障碍物,融合后的信息
    DPoint2s       obs_fuse_[MAX_OBSNUMBER_CONST];
};

#ifdef SIMULATION
class Teammatesinfo_sim          // This is for simulation only. Not used now.
{
public:
    Teammatesinfo info_sim[5];
};
#endif

}

#endif // _NUBOT_TEAMMATESINFO_H
