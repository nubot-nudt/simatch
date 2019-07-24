#ifndef _NUBOT_BALL_H_
#define _NUBOT_BALL_H_

#include "world_model/robot.h"
#include "ros/ros.h"

namespace nubot{

const int  BALL_PREDICT_BIAS_CONST=60;
const int  THRES_BALL_VALID_CONST=15;
enum {NOTSEEBALL = 0, SEEBALLBYOWN = 1,SEEBALLBYOTHERS = 2};
enum {OMNI_BALL  = 0, KINECT_BALL  = 1};
class BallObject{

public:
    BallObject(int id=-1,bool is_valid=false,DPoint loc=DPoint(0,0),
               PPoint real_loc=PPoint(Angle(0),0.0),
               DPoint velocity=DPoint(0,0), bool pos_known=false,
               bool   vec_known=false, double lifetime = -230435);
	
    BallObject (const BallObject& _info) ;
	
    const BallObject & operator=(const BallObject & _info)
	{
        robot_id_        = _info.robot_id_;
        ball_global_loc_ = _info.ball_global_loc_;
        ball_real_loc_   = _info.ball_real_loc_;
        is_ball_valid_   = _info.is_ball_valid_;
        ball_velocity_   = _info.ball_velocity_;
        ball_pos_known_  = _info.ball_pos_known_;
        ball_vec_known_  = _info.ball_vec_known_;
		return *this;
	}

    void setID(int _id);
    void setValid(bool is_valid);
    void setGlobalLocation(DPoint _loc);
    void setRealLocation(PPoint _loc);
    void setVelocity(DPoint _vec);
    void setLocationKnown(bool pos_known);
    void setVelocityKnown(bool vec_known);
    void setlifetime(double lifetime);

    int    getID();
    DPoint getGlobalLocation();
    PPoint getRealLocation();
    DPoint getVelocity();
    bool isValid();
    bool isLocationKnown();
    bool isVelocityKnown();
    double getlifetime();

private:

    int       robot_id_;          /** 表示机器人的编号，多机器人通信*/
    bool      is_ball_valid_;     /** 足球信息是更新，长时间未更新可能置为false*/
    PPoint    ball_real_loc_;     /** 足球的在机器人体坐标系下坐标，多机器人系统记住该值需要重新计算*/
    DPoint    ball_global_loc_;   /** 足球的在世界坐标系下坐标*/
    DPoint    ball_velocity_;     /** 足球的在世界坐标系下速度*/
    bool      ball_pos_known_;    /** 足球的位置是否已知 */
    bool      ball_vec_known_;    /** 足球的速度是否已知*/
    double    lifetime_;          /** 足球更新时间，用于判断is_ball_valid_是否有效*/
};


class Ball{

public:
    Ball(void);
    ~Ball(void);

    /** @brief 更新球的信息，并将不同球的信息进行融合 */
    void
    update(BallObject & teammates_ball, bool is_valid);


    bool
    evaluateVelocity(std::vector<BallObject> & _ball_info,
                     std::vector<ros::Time> & _count_time,int & _nums_predict_errors);

public:

    /** 感知到的足球信息
     *  OMNI_BALL=0，全向视觉系统检测到的足球
     *  KINECT_BALL=1，kinect传感器检测到的足球信息
     */
    std::vector< BallObject > sensor_ball_;
    /** @brief kinect传感器检测到的足球信息*/
    BallObject own_ball_;

    /** @brief 将全向视觉系统、前向视觉系统、kinect、队友的足球进行融合，得到最终的足球信息 */
    BallObject fuse_ball_;
    int  ball_info_state_;   /** 足球的状态，主要是用在多机器人协同方面，表示足球状态*/

    /** @brief 记录全向视觉系统得到的足球信息，用于球速估计*/
    std::vector<BallObject> omni_ball_record_;
    std::vector<ros::Time>  omni_ball_time_;
    /** @brief 记录kinect传感器得到的足球信息，用于球速估计*/
    std::vector<BallObject> kinect_ball_record_;
    std::vector<ros::Time>  kinect_ball_time_;
};

}



#endif
