#ifndef _NUBOT_BEHAVIOUR_H
#define _NUBOT_BEHAVIOUR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string.h>
#include "core.hpp"
#include "common.hpp"

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

using namespace std;

namespace nubot{

/** Behaviour主要是底层的控制函数，控制机器人按照预定的轨迹运动*/

class Behaviour
{

public:
    Behaviour();
    ~ Behaviour();

    float basicPDControl(float pgain,float dgain, float err,float err1, float maxval);
    float basicPIDcontrol(float pgain,
                       float igain,
                       float dgain,
                       float currval,
                       float targetval,
                       float imaxlimiter,
                       float iminlimiter,
                       float& dstate,
                       float& istate );
    void fuzzyPIDcontrol(float &deltakp, float &deltaki,float &deltakd, float err,float err1);

    void move2PositionForDiff(float kp, float kalpha, float kbeta, DPoint target, float maxvel,
                              const DPoint  & _robot_pos,const Angle  & _robot_ori );
    void move2PositionForDiff(float kp, float kalpha, float kbeta, DPoint target0, DPoint target1,
                              float maxvel, const DPoint & _robot_pos,const Angle  & _robot_ori, const DPoint & _robot_vel);
    /** 采用PD控制运动到目标点*/
    void move2Position(float pval, float dval, DPoint target, float maxvel,
                       const DPoint  & _robot_pos,const Angle  & _robot_ori );
    void move2target(float pval, float dval,DPoint target, DPoint realtarvel, float maxvel,
                     const DPoint  & _robot_pos,const Angle  & _robot_ori);
    void traceTarget();
    void revDecoupleFromVel(float vx,float vy,float &positivemax_rev,float &negativemax_rev);
    /** rotate to the target orientation by using PD control*/
    void rotate2AbsOrienation(float pval, float dval, float orientation,float maxw,const Angle & _robot_ori);
    void rotate2RelOrienation(float pval, float dval, float rel_orientation,float maxw);
    void rotatetowardsSetPoint(DPoint point);
    void rotatetowardsRelPoint(DPoint rel_point);
    void clearBehaviorState();
    void setAppvx(double vx);
    void setAppvy(double vy);
    void setAppw(double w);
    void setTurn(bool isTurn);
    void accelerateLimit(const double &_acc_thresh = 20, const bool & use_convected_acc = true);
    void clear();
public:
    float app_vx_;
    float app_vy_;
    float app_w_;
    bool  isTurn_;
    float last_app_vx_;
    float last_app_vy_;
    float last_app_w_;
    float last_speed;
 };
}

#endif // _NUBOT_BEHAVIOUR_H
