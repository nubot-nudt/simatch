#ifndef NUBOT_ROBOT_H_
#define NUBOT_ROBOT_H_

#include "core.hpp"

namespace nubot{

const int THRES_ROBOT_VALID_CONST=15;

template<typename _T> struct Point_
{
  _T x;
  _T y;
};

typedef Point_<double>  Point2d;

typedef Point2d Point;

struct obs_info_zip
{
    double x,y;
    double HRZ[4];
    double HRH[4];
};

const int MAX_OBSTALCES_NUMS_CONST = 9;

class Robot
{

public:

    Robot(int id=-1,int num=-1,int catch_num=0,int pass_num=0, DPoint loc=DPoint(0,0),Angle head=Angle(0),DPoint vec=DPoint(0,0),
          double w=0.0, bool is_kick_off=false,bool is_robot_stuck=false,
          bool is_robot_slip=false, bool isvalid=false, double lifetime = -2235432,
          char _current_role = NOROLE, DPoint _target = DPoint(0,0),
          bool is_robot_dribble=false,double role_preserve = 0);

    Robot (const Robot& _info) ;

    const Robot& operator= (const Robot& _info)
	{
		robot_id_   = _info.robot_id_;
        for(int i=0;i<OUR_TEAM;i++)
            target_num_[i]=_info.target_num_[i];
        staticcatch_num_=_info.staticcatch_num_;
        staticpass_num_=_info.staticpass_num_;
		robot_loc_  = _info.robot_loc_;
		robot_head_ = _info.robot_head_;
		robot_vec_  = _info.robot_vec_;
		robot_w_    = _info.robot_w_;
		is_kick_off_= _info.is_kick_off_;
	    is_robot_valid_ = _info.is_robot_valid_;
        is_robot_stuck_ = _info.is_robot_stuck_;
        is_robot_slip_  = _info.is_robot_slip_;
        lifetime_   = _info.lifetime_;
        current_role_ = _info.current_role_;
        target_     = _info.target_;
        is_robot_dribble_ = _info.is_robot_dribble_;
        role_preserve_time_ = _info.role_preserve_time_;
        current_action_ = _info.current_action_;
		return *this;
	}

    ~Robot(void);
    bool isValid();
    void setValid(bool _isvalid);
    void setID(int _id);
    void setTargetNum(int _id,int _num);
    void setcatchNum(int _num);
    void setpassNum(int _num);
    void setLocation(const DPoint & _loc);
    void setHead(const Angle &  _head);
    void setVelocity(const DPoint & _vec);
    void setW(double _w);
    void setKick(bool _iskick);
    void setStuck(bool _isstuck);
    void setSlip(bool _isslip);
    void setlifetime(double lifetime);
    void setTarget(const DPoint &  _target);
    DPoint getTarget();
    void setCurrentRole(char _current_role);
    char getCurrentRole();
    void setCurrentAction(char _current_action);
    char getCurrentAction();
    void setDribbleState(bool _isdribble);
    bool getDribbleState();
    int  getID();
    int  getTargetNum(int _id);
    int    getcatchNum();
    int    getpassNum();
    DPoint getLocation();
    Angle  getHead();
    DPoint getVelocity();
    DPoint getAcc();
    double getW();
    bool   isKickoff();
    bool   isStuck();
    bool   isSlip();
    double getlifetime();
    void   update();
    double getRolePreserveTime();
    void   setRolePreserveTime(double _preserve_time);
private:
    int      robot_id_;       /** @brief 机器人ID编号，根据.bashrc中的AGENT 变量 */
    int      target_num_[OUR_TEAM];     /** 静态站位的目标点编号，用于处理冲突 */
    int      staticpass_num_;          /** 静态传接球的传球机器人编号 */
    int      staticcatch_num_;         /** 静态传接球的接球机器人编号 */
    DPoint   robot_loc_;      /** @brief 机器人在世界坐标系下坐标 */
    Angle    robot_head_;     /** @brief 机器人在世界坐标系下朝向 */
    DPoint   robot_vec_;      /** @brief 机器人在世界坐标系下速度 */
    double   robot_w_;        /** @brief t机器人在世界坐标系下角速度 */
    bool     is_kick_off_;    /** @brief 机器人是否将球踢出 */
    bool     is_robot_stuck_; /** @brief 机器人是否堵转 */
    bool     is_robot_dribble_; /** @brief 机器人是否堵转 */
    bool     is_robot_slip_;  /** @brief 机器人是否打滑 */
    bool     is_robot_valid_; /** @brief 机器人信息是否有效，主要是队友通信是需要有通信延迟判断 */
    double   lifetime_;       /** @brief 机器人接收到的视觉topic时间间隔（自己），队友的是通信两帧之间延时间隔 */
    char     current_role_;   /** @brief 机器人当前角色，来源于上层控制节点 */
    char     current_action_;
    DPoint   target_;         /** @brief 机器人当前的跑位目标点*/
    double   role_preserve_time_; /** @ 当前角色保持的时间*/
};

}

#endif // ROBOTINFO_H
