#include "world_model/robot.h"

using namespace nubot;
Robot::Robot(int id, int num, int catch_num, int pass_num, DPoint loc, Angle head, DPoint vec, double w,
             bool  is_kick_off, bool is_robot_stuck, bool is_robot_slip,
             bool isvalid, double lifetime, char _current_role,
             DPoint _target, bool is_robot_dribble, double role_preserve)
{
    robot_id_  = id;
    robot_loc_ = loc;
    robot_head_= head;
    robot_vec_ = vec;
    robot_w_   = w;
    is_kick_off_    = is_kick_off;
    is_robot_stuck_ = is_robot_stuck;
    is_robot_slip_  = is_robot_slip;
    is_robot_valid_ = isvalid;
    lifetime_ = lifetime;
    current_role_ = _current_role;
    target_       = _target;
    is_robot_dribble_ = is_robot_dribble;
    role_preserve_time_ = role_preserve;
    for(int i=0;i<OUR_TEAM;i++)
        target_num_[i]=num;
    staticcatch_num_=catch_num;
    staticpass_num_=pass_num;
}

Robot::Robot(const Robot&_info)
{
    robot_id_   = _info.robot_id_;
    for(int i=0;i<OUR_TEAM;i++)
        target_num_[i]=_info.target_num_[i];
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
}
Robot::~Robot() { }
void
Robot::setID(int _id)                { robot_id_=_id;  }
void
Robot::setTargetNum(int _id, int _num)        { target_num_[_id]=_num;}
void
Robot::setcatchNum(int _num)                { staticcatch_num_=_num;  }
void
Robot::setpassNum(int _num)                { staticpass_num_=_num;  }
void
Robot::setValid(bool _isvalid)       { is_robot_valid_ = _isvalid;}
void
Robot::setLocation(const DPoint & _loc)      { robot_loc_=_loc; }
void
Robot::setHead(const Angle  & _head)          { robot_head_=_head;}
void
Robot::setVelocity(const DPoint  & _vec)      { robot_vec_=_vec;}
void
Robot::setW(double _w)               { robot_w_=_w;}

void
Robot::setKick(bool _iskick)         { is_kick_off_=_iskick;}
void
Robot::setSlip(bool _isslip)         { is_robot_slip_=_isslip;}
void
Robot::setStuck(bool _isstuck)       { is_robot_stuck_=_isstuck;}
void
Robot::setlifetime(double lifetime)     { lifetime_  = lifetime;}
int
Robot::getID()         { return robot_id_;}
int
Robot::getTargetNum(int _id)  { return target_num_[_id];}
int
Robot::getcatchNum()                { return staticcatch_num_;  }
int
Robot::getpassNum()                { return staticpass_num_;  }
DPoint
Robot::getLocation()   { return robot_loc_;}
Angle
Robot::getHead()       { return robot_head_;}
DPoint
Robot::getVelocity()   { return robot_vec_;}
double
Robot::getW()          { return robot_w_;}
bool
Robot::isKickoff()     { return is_kick_off_;}
bool
Robot::isStuck()       { return is_robot_stuck_;}
bool
Robot::isSlip()        { return is_robot_slip_;}
bool
Robot::isValid()       { return is_robot_valid_;}
double
Robot::getlifetime()   { return lifetime_;}
void
Robot::setTarget(const DPoint & _target) { target_ = _target;}
DPoint
Robot::getTarget()      { return target_;}
void
Robot::setCurrentRole(char _current_role)  { current_role_ = _current_role;}
char
Robot::getCurrentRole() {return current_role_;}
void
Robot::setCurrentAction(char _current_action){ current_action_ = _current_action;}
char
Robot::getCurrentAction(){return current_action_;}
void
Robot::setDribbleState(bool _isdribble)  { is_robot_dribble_ = _isdribble;}
bool
Robot::getDribbleState() {return is_robot_dribble_;}
void
Robot::setRolePreserveTime(double _preserve_time)  { role_preserve_time_ = _preserve_time;}
double
Robot::getRolePreserveTime() {return role_preserve_time_;}
/** 当机器人断电时，is_robot_valid_=false，每次更新的初始值是来源于队友开关电状态，
 *  这时不管lifetime怎么样都应该处于无效状态. 当机器人is_robot_valid_=true，
 *  可以表示机器人当前通电，然后考虑通信延迟问题，如果时间太长没有更新，认为信息失效.
 */
void
Robot::update()
{
    if(lifetime_ > NOT_DATAUPDATE || lifetime_ < 0 || is_robot_valid_==false )
        is_robot_valid_ = false;
    else
        is_robot_valid_ = true;
}


