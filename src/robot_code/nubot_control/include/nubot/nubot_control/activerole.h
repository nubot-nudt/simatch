#ifndef _NUBOT_ACTIVEROLE_H
#define _NUBOT_ACTIVEROLE_H

#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/fieldinformation.h>
#include <nubot/nubot_control/plan.h>
namespace nubot {

class ActiveRole
{
public:
      ActiveRole();
     ~ActiveRole();
void process();
bool checkPass();
void clearActiveState();
bool isNullInTrap(const std::vector<DPoint> & obs_info,const DPoint & robot_pos, const Angle & direction = Angle(0),
                  double back_width = 50, double front_width = 75, double back_len = -25, double front_len = 100);
bool pnpoly(const std::vector<DPoint> & pts, const std::vector<DPoint> & obs_pts);
void activeDecisionMaking();
void selectCurrentState();
void selectCurrentAction(unsigned char state);
bool evaluateKick(DPoint & kick_target,Angle & leftdelta,Angle &rightdelta);
void caculatePassEnergy(double & energy, int & label);
void caculateDribblingEnergy(double & avoid_enegy,bool isNullFrontRobot);
void selectDribblingOrPassing(bool isNullFrontRobot);
void findBall();
void turn4Shoot(DPoint kicktarget);
void NewAvoidObs();
void NewAvoidObsForPass();
void activeCatchBall();
void triggerShoot(DPoint target);
void stuckProcess();
void kickball4Coop(DPoint target);
bool IsLocationInOppGoalArea(DPoint location);
bool IsLocationInField(DPoint location);
bool IsLocationInOppPenalty(DPoint location);
bool IsLocationInOurPenalty(DPoint location);
bool IsLocationInOppField(DPoint location);
bool IsLocationInOurField(DPoint location);
public:
    World_Model_Info * world_model_;
    Plan  * m_plan_;
    bool stuckflg_ ;             /** 机器人当前是否堵转*/
    int  stucktime_ ;            /** 机器人堵转周期记录*/
    bool NeedEvaluat ;           /** 射门的ROS服务，于底层控制节点通信*/
    bool dynamic_shoot_state_ ;  /** 动态射门的状态*/
    int  dynamic_shoot_count_ ;  /** 动态射门的*/
    bool quick_shoot_state_ ;    /** 快速射门状态*/
    int  quick_shoot_count_ ;
    int  pass_lock_;
    bool catch_in_ourfeild_;
    DPoint target_shoot_;
    Angle  ldetAng_shoot_;
    Angle  rdetAng_shoot_;
    unsigned char currentstate_;
    bool   kick_enable_;         /** 准备踢球*/
    bool   dribble_enable_;      /** 带球状态*/
    float kick_force_;          /** 踢球的力量*/
    DPoint kick_target_;
};

}


#endif // _NUBOT_ACTIVEROLE_H
