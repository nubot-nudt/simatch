#ifndef _NUBOT_TEAMMATESINFO_H
#define _NUBOT_TEAMMATESINFO_H

#include "nubot/world_model/ball.h"
#include "nubot/world_model/robot.h"
#include "nubot/world_model/obstacles.h"
namespace nubot{

class PassCommands //is_pass = true,准备接球，每个机器人都收到，就可以判断状态了
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

class Teammatesinfo
{
public:
    Robot          robot_info_;
    BallObject     ball_info_;
    ObstacleObject obs_info_[MAX_OBSNUMBER_CONST];
    PassCommands   pass_cmds_;
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
