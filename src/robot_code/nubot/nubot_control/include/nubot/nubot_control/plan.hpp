#ifndef PLAN_H
#define PLAN_H

#include <cmath>
#include "nubot/core/core.hpp"
#include "nubot/nubot_control/behaviour.hpp"
#include <boost/ptr_container/ptr_list.hpp>

using namespace std;
namespace nubot{
class WolrdModeliInfo
{
    public:
           WolrdModeliInfo()
               :active_robot_num_(0),is_robot_stuck_(false),ball_info_state_(0),
                 game_ctrl_(CTRL_STOP),indist_(0)
           {
               robot_pos_ = DPoint(0.0,0.0);
               robot_ori_ = Angle(0.0);
               robot_vel_ = DPoint(0.0,0.0);
               ball_pos_  = DPoint(0.0,0.0);
               ball_vel_  = DPoint(0.0,0.0);
               obs_pos_.reserve(10);
               target_    = DPoint(0.0,0.0);
               target_orientation_ = 0.0;
           }
           ~WolrdModeliInfo()
           {}
    public:

           int    active_robot_num_;   //current active robot number

           DPoint robot_pos_;          //robot position
           Angle  robot_ori_;          //robot orientation
           DPoint robot_vel_;          //robot velocity
           bool   is_robot_stuck_;     //stuck check

           DPoint ball_pos_;           //ball position
           DPoint ball_vel_;           //ball velocity
           int    ball_info_state_;    //ball information state    local ?  share ? or cannot see


           std::vector<DPoint> obs_pos_;            //obstacle information

           unsigned char game_ctrl_;   //current order

           double    indist_;
           DPoint    target_;             //target from coach
           float     target_orientation_; //the orientation of the specified target


};

    class Plan
    {
    public:
        Plan();
        /*******************catch ball******************/
        void 
        traceBall();      // trace
        void
        interceptBall();  // intercept

        void
        catchBall();
        void
        catchBallSlowly();
        void
        catchBallForCoop();
        void
        catchMovingBall();
        void 
        catchMotionlessBall();

        void
        positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta);

        DPoint
        avoidRelOble(DPoint target, double theta, double ro, double vel, bool includeball);
        void
        driblleControl(DPoint target,double acc,double sacc,double lvel,double maxvel);

        void
            move2Positionwithobs(DPoint target, float maxvel, float maxacc);  // move to the target point with obstacles avoidance
        void
            move2Positionwithobs_noball(DPoint target, float maxvel, float maxacc, bool avoid_ball=false);
        //subtargets
        int
        Min_num(int n,double *q);
        double
        Min(int n,double *q);
        int
        Max_num(int n,double *q);
        double
        Max(int n,double *q);
        void
        subtarget(double *obs,double *pos_robot,double *pos_target,double *pos_subtarget);
        void
        subtargets(DPoint target,DPoint robotpos);
        void
        subtargets_withball(DPoint target,DPoint robotpos);
        void
        subtargets2(DPoint target,DPoint robotpos);



        double
               CaculatediffS(double &s, double &rho,double &phid,
                             double v0,double c ,  double alpha ,
                             double p,double q,double  diffp,double diffq);
               double
               CaculatediffS(double &s, double &rho,double &phid,
                             double lamda,double v0,double p,
                             double q,double  diffp,double diffq);
               double
               CaculatePhid(double  rho,double basic_phid ,
                            double thetar , double eps);
              

              double
               Curvature(double diffp , double diffq, double double_diffp,double double_diffq);

               


           boost::ptr_list<DPoint> Active_ControlPointList;
               boost::ptr_list<DPoint> Passive_ControlPointList;

               double
               Bernstein(double s , int i , int N);
               void
               Bezier(double s,boost::ptr_list<DPoint> &controlpoint,
                      double &p ,double &q, double &diffp,double &diffq,
                      double &double_diffp, double &double_diffq); //  ¿ØÖÆµã £¬ ¿ŽÊÇŒžœ×µÄbezierÇúÏß ...

               void
               Bezier(double s,boost::ptr_list<DPoint> &controlpoint,
                      double &p ,double &q, double &diffp,double &diffq);
               bool
               BezierPathFollow( boost::ptr_list<DPoint> &controlpoint ,
                                 double v0,double c , double alpha ,double k,
                                 double lamda ,double eps); //s
               void
               FromPath2Trajectory(double &s,double &rtheta ,double vr,
                                   double wr,double diffp,double diffq,
                                   double double_diffp,double double_diffq,
                                   double vprofile ,double aprofile);//
               //bool
               //BezierTrajectoryTracking4Pass(boost::ptr_list<DPoint> &controlpoint ,double v0,double a);
                bool
                BezierTrajectoryTracking(boost::ptr_list<DPoint> &controlpoint ,double v0,double a);

                double
                PECrossBackMIdlleLine(double direction);
                double
                PEOutField(double diretion);
                double
                PEInOurPenaty(double direction);
                double
                PObleDirection4OurField(double direction, double predictlen,double cobledirection,
                                               double kobledirection);


                bool
                SearchMinPE4PassThroughforOurField(double &direction,double pridictlen,
                                                          DPoint trap[4],double step,int flg);

                bool
                SearchMinPE4PassThrough(double &direction,double pridictlen,
                                        DPoint trap[4],double step,int flg);




                bool
                IsNullInTrap(double direction,double swidth,double lwidth,double len);
                bool
                checkinOurField(DPoint mypos);
                bool
                checkinOppField(DPoint mypos);
                bool
                checkinOurPenalty(DPoint object);
                double
                PObleDirection(double direction, double predictlen, double cobledirection,double kobledirection);

                DPoint
                FindBstDirectionForAvoid(DPoint target);
                double
                FindBstDirectionForAvoid();
                double
                FindBstDirectionForAvoid2(DPoint target);
                double
                SearchDirectionforMinPEPoint(double oridirection,double step,int lefttime,int righttime);
                int
                GetAvoidState();

    public:
        Behaviour m_behaviour_;
        WolrdModeliInfo* worldmodelinfo_;

        DPoint   subtargets_pos_;
        DPoint   subtargets2_pos_;

        bool inourfield_;
        bool inoppfield_;
        double lastdirection;
        double pe1_;
        double pe2_;
        bool   isnull_;


        double bezier_s;
        bool   bezier_updateflag;
        double bezier_vr ,bezier_wr ;
        double bezier_vm ;
        double bezier_p ,bezier_q ,bezier_rtheta ;  //  ŽËŽŠÓÃµÄÊÇstatic £¬±ØÐëžÄ±ä²ÅÐÐ
        double last_bezier_rtheta;

        float kp;
        float kalpha;
        float kbeta;

        vector<DPoint> target_;

    public:
        bool   isinposition_;

    };
}
#endif // PLAN_H
