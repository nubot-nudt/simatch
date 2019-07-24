#ifndef PLAN_H
#define PLAN_H

#include <cmath>
#include "core.hpp"
#include "nubot/nubot_control/subtargets.h"
#include "nubot/nubot_control/behaviour.hpp"
#include "nubot/nubot_control/world_model_info.h"

using namespace std;
namespace nubot{
class Plan
{
public:
        Plan();

        /***********catch ball***********/
        void catchBall();
        void catchBallForCoop();
        void catchBallSlowly();
        void catchMovingBall();
        void catchMotionlessBall();

        /***********postion*************/
        void positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta);
        void driblleControl(DPoint target,double acc,double sacc,double lvel,double maxvel);
        //void move2Positionwithobs(DPoint target);  // move to the target point with obstacles avoidance
        void move2Positionwithobs_noball(DPoint target, float maxvel, float maxacc, bool avoid_ball=false);

        /***********PE and PO***********/
        double PECrossBackMIdlleLine(double direction);
        double PEOutField(double direction);
        double PEInOurPenaty(double direction);
        double PObleDirection4OurField(double direction, double predictlen, double cobledirection, double kobledirection);
        double PObleDirection(double direction, double predictlen, double cobledirection,double kobledirection);

        /***********check***********/
        bool IsNullInTrap(double direction, double swidth, double lwidth, double len);
        /*********find and search*********/
        double FindBstDirectionForAvoid();
        double FindBstDirectionForAvoid2(DPoint target);
        int    GetAvoidState();
        double SearchDirectionforMinPEPoint(double oridirection,double step,int lefttime,int righttime);
        bool   SearchMinPE4PassThroughforOurField(double &direction,double pridictlen,DPoint trap[4],double step,int flg);
        bool   SearchMinPE4PassThrough(double &direction, double pridictlen, DPoint trap[4], double step, int flg);

        void   update();
public:
        World_Model_Info * world_model_;
        Behaviour  m_behaviour_;
        Subtargets m_subtargets_;

        float kp;
        float kalpha;
        float kbeta;
        bool  inourfield_;
        bool  inoppfield_;
        double lastdirection;

        DPoint robot_pos_;
        Angle  robot_ori_;
        DPoint robot_vec_;
        DPoint ball_pos_;
        DPoint ball_vec_;

        vector<DPoint> target_;

public:
        bool   isinposition_;
};
}
#endif //PLAN_H
