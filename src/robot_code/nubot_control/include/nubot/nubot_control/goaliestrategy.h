#ifndef GOALIE_STRATEGY_H
#define GOALIE_STRATEGY_H
//ROS includes
#include <ros/ros.h>

//nubot includes
#include "core.hpp"
#include <nubot_common/BallInfo3d.h>
#include <nubot_common/OminiVisionInfo.h>

namespace nubot
{
  #define HafeGravity		490.0			//cm/s^2
  #define MAXNUM_OF_FIT	        15			//max num of data points, after the amount is out of size, the oldest data will be auto overlaped
  #define GOAL_POS_X            (-858)                //the position of our goal line is -900
  #define GOAL_POS_Y        (100-30)        //goal is -100~100, robot width is 70
  class ParabolaFitter3D
  {
  //------------------User Manual---------------
  //for every data point you get, you can call the "FlyCheckAndAddData" function, it will return you if the ball is upper the ground.
  //if so, the data will be auto saved, else the data buffer will be auto cleared.
  //you can check the "n" of this class, which means the number of points(of course upper the ground) saved,
  //and you can call "Fitting" to fit the points at any time(of course when "n"<3, it makes no sence).
  //the fitting result is saved in model_param_[6] which means: x=p0*t+p1, y=p2*t+p3, z=-HafeGravity*t^2+p4*t+p5
  //at any time you can call "clearDataBuffer", but usually you will do this after you have done the work associate with the current flight of ball.
  //all the unit used in this class is "cm" and "s", so make sure your data is currect.
  //--------------------------------------------

  public:
    double model_param_[6];//fitting formula: x=p0*t+p1, y=p2*t+p3, z=-HafeGravity*t^2+p4*t+p5
    DPoint bounding_point_;
    double bounding_time_;
    DPoint crossing_point_;
    double crossing_time_;
    int n_;                //the number of points(of course upper the ground) saved, (0~MAXNUM_OF_FIT)
  private:
    double t_[MAXNUM_OF_FIT];//data buffer
    double x_[MAXNUM_OF_FIT];//data buffer
    double y_[MAXNUM_OF_FIT];//data buffer
    double z_[MAXNUM_OF_FIT];//data buffer
    int data_pointer_;//data pointer of the data buffer, when no data -1, one data 0, ..
    int fly_flag_; //used for filtering when check if the ball is upper the ground
    double z_old_;//used for filtering when check if the ball is upper the ground
    double t_old_;//used for filtering when check if the ball is upper the ground
    double sum_t_,sum_x_,sum_y_,sum_z_,sum_tt_,sum_tx_,sum_ty_,sum_tz_,t0_;//used for fitting
    int addData(const double _add_z, const DPoint& _add_pos_now, const double _time);//be called in "FlyCheckAndAddData"
  public:
    ParabolaFitter3D();
    bool flyCheckAndAddData(const double _z_now, const DPoint& _pos_now, const double _time);
    void clearDataBuffer();
    void fitting(double* _pfitting_err=NULL);
    void saveFileTXT(const char* _pFilename);//save data buffer and fitting result
    double getStartTime();
    double getEndTime();
  };

  class GoalieStrategy
  {
  public:
    typedef enum {
      StandBy      = 0,
      Move2Ball    = 1,
      Move2Origin  = 2,
      Turn2Ball    = 3,
//    CatchBall    = 4,
//    KickBall     = 5,
    } GoalieState;//the goalie state of FSM
    GoalieState state_;
    DPoint dest_point_;  //Destination made by strategy()
    double dest_angle_;  //Destination made by strategy()
    double thresh_vel_;  //thresh of movement made by strategy()
    double thresh_omiga_;//thresh of movement made by strategy()
    std::string debug_str_;//the info of goalie set by strategy()

    GoalieStrategy();
    nubot_common::RobotInfo       robot_info_;
    nubot_common::BallInfo3d      ball_info1_3d_;//you can set ball_info_3d_ using global info, or call setBallInfo3dRel() using relative info
    nubot_common::BallInfo3d      ball_info2_3d_;//you can set ball_info_3d_ using global info, or call setBallInfo3dRel() using relative info
    nubot_common::BallInfo3d ball_info_3d_;//if kinect#1 see ball, it will be set to #1; else set to #2 
    nubot_common::BallInfo        ball_info_2d_;
    void setBallInfo3dRel(const nubot_common::BallInfo3d &_robot_info_3d_rel);//you can set ball_info_3d_ using global info, or call setBallInfo3dRel() using relative info
    bool ballTrack( const int THRESH_GROUND_VEL=200, const bool use_parabola_fitter_=true );//used in "mainCallBack()", and make a prediction if ball could goal
    void strategy();//used in "mainCallBack()", make decision using prediction info

  private:
    ParabolaFitter3D parabola_fitter_;//used in ballTrack()
    bool   predicted_3d_;             //set by ballTrack()
    bool   predicted_2d_;             //set by ballTrack()
    bool   predictec_omi_;            //set by ballTrack()
    DPoint bounding_pt_;              //set by ballTrack()
    DPoint crossing_pt_;              //set by ballTrack()
    double bounding_time_;            //set by ballTrack()
    double crossing_time_dis_;            //set by ballTrack()
    double time_now_;                 //set by ballTrack()    
  };
 
}

#endif
