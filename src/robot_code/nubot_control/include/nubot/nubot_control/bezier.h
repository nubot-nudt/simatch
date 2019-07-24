#ifndef BEZIER_H
#define BEZIER_H

#include <cmath>
#include "core.hpp"
#include "nubot/nubot_control/behaviour.hpp"
#include "nubot/nubot_control/world_model_info.h"
#include <boost/ptr_container/ptr_list.hpp>

using namespace std;
namespace nubot{
class Bezier
{
public:
        Bezier(double s,boost::ptr_list<DPoint> &controlpoint,double &p ,double &q, double &diffp,double &diffq,double &double_diffp, double &double_diffq);
        Bezier(double s,boost::ptr_list<DPoint> &controlpoint,double &p ,double &q, double &diffp,double &diffq);

        double CalculatediffS(double &s,double &rho,double &phid,double v0,double c,double alpha,double p,double q,double diffp,double diffq);
        double CalculatediffS(double &s, double &rho,double &phid,double lamda,double v0,double p,double q,double  diffp,double diffq);
        double CalculatePhid(double rho, double basic_phid, double thetar, double eps);
        double Curvature(double diffp, double diffq, double double_diffp, double double_diffq);
        double Bernstein(double s , int i , int N);

        bool BezierPathFollow(boost::ptr_list<DPoint> &controlpoint,double v0,double c , double alpha ,double k, double lamda);
        void FromPath2Trajectory(double &s,double &rtheta ,double vr,double wr,double diffp,double diffq,double double_diffp,double double_diffq,double vprofile ,double aprofile);
        bool BezierTrajectoryTracking(boost::ptr_list<DPoint> &controlpoint,double v0,double a);
public:
        Behaviour * behaviour_;
        World_Model_Info * world_model_;

        double bezier_s;
        bool   bezier_updateflag;
        double bezier_vr ,bezier_wr ;
        double bezier_vm ;
        double bezier_p ,bezier_q ,bezier_rtheta ;
        double last_bezier_rtheta;

        boost::ptr_list<DPoint> Active_ControlPointList;
        boost::ptr_list<DPoint> Passive_ControlPointList;

};
}
#endif //BEZIER_H
