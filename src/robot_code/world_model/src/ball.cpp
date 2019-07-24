#include "world_model/world_model.h"
using namespace nubot;

BallObject::BallObject(int id,bool is_valid,DPoint loc ,PPoint real_loc,
                       DPoint velocity, bool pos_known,
                       bool vec_known, double lifetime)
{
    robot_id_        = id;
    ball_global_loc_ = loc;
    ball_real_loc_   = real_loc;
    is_ball_valid_   = is_valid;
    ball_velocity_   = velocity;
    ball_pos_known_  = pos_known;
    ball_vec_known_  = vec_known;
    lifetime_        = lifetime;
}
BallObject::BallObject (const BallObject& _info)
{
    robot_id_        = _info.robot_id_;
    ball_global_loc_ = _info.ball_global_loc_;
    ball_real_loc_   = _info.ball_real_loc_;
    is_ball_valid_   = _info.is_ball_valid_;
    ball_velocity_   = _info.ball_velocity_;
    ball_pos_known_  = _info.ball_pos_known_;
    ball_vec_known_  = _info.ball_vec_known_;
    lifetime_        = _info.lifetime_;
}

void
BallObject::setID(int _id)                     { robot_id_=_id; }
void
BallObject::setValid(bool is_valid)            { is_ball_valid_=is_valid; }
void
BallObject::setGlobalLocation(DPoint _loc)     { ball_global_loc_=_loc; }
void
BallObject::setRealLocation(PPoint _loc)       { ball_real_loc_=_loc; }
void
BallObject::setVelocity(DPoint _vec)           { ball_velocity_=_vec; }
void
BallObject::setLocationKnown(bool pos_known)   { ball_pos_known_=pos_known;}
void
BallObject::setVelocityKnown(bool vec_known)   { ball_vec_known_=vec_known; }
void
BallObject::setlifetime(double lifetime)       { lifetime_ = lifetime; }

int
BallObject::getID()             { return robot_id_;}
DPoint
BallObject::getGlobalLocation() { return ball_global_loc_;}
PPoint
BallObject::getRealLocation()   { return ball_real_loc_;}
DPoint
BallObject::getVelocity()       { return ball_velocity_;}
bool
BallObject::isValid()           { return is_ball_valid_;}
bool
BallObject::isLocationKnown()   { return ball_pos_known_;}
bool
BallObject::isVelocityKnown()   { return ball_vec_known_;}
double
BallObject::getlifetime()       { return lifetime_;}

/// \brief 感知到的足球信息
/// OMNI_BALL=0  ， 全向视觉系统检测到的足球
/// KINECT_BALL=1， kinect传感器检测到的足球信息
Ball::Ball(void)
{
    sensor_ball_.resize(2);

    omni_ball_record_.reserve(25);
    omni_ball_time_.reserve(25);

    kinect_ball_record_.reserve(25);
    kinect_ball_time_.reserve(25);
}
Ball::~Ball(void)
{
}

/// \brief 更新自身传感器的信息，并进行融合
void
Ball::update(BallObject & teammates_ball,bool is_valid)
{
    own_ball_.setValid(false);
    own_ball_.setLocationKnown(false);
    own_ball_.setVelocityKnown(false);
    own_ball_.setVelocity(DPoint(0,0));
    ball_info_state_ = NOTSEEBALL;
    DPoint velocity    = own_ball_.getVelocity();
    bool   is_velocity = own_ball_.isVelocityKnown();

    BallObject & kinect_ball_info = sensor_ball_[KINECT_BALL];
    double ltime = kinect_ball_info.getlifetime();
    //printf("ball ball kinect angle: %d radians: %f\n",kinect_ball_info.getRealLocation().angle_.degree(),kinect_ball_info.getRealLocation().radius_);
    bool flagkinect=false;
    bool flagomni=false;
    /// 长时间没有更新数据，则判定起为false
    if(ltime < 0 || ltime > NOT_DATAUPDATE)
        kinect_ball_info.setValid(false);

    if (kinect_ball_info.isValid() && kinect_ball_info.isLocationKnown())
            flagkinect=true;
    BallObject &omni_ball_info = sensor_ball_[OMNI_BALL];
    /// kinect检测到的球的全局坐标
    DPoint kk=kinect_ball_info.getGlobalLocation();
    /// 全向视觉检测到的球的全局坐标
    DPoint ko=omni_ball_info.getGlobalLocation();

    double ltime1 = omni_ball_info.getlifetime();
    /// 长时间没有更新数据，则判定起为false
    if(ltime1 < 0 || ltime1 > NOT_DATAUPDATE)
        omni_ball_info.setValid(false);
    if (omni_ball_info.isValid() && omni_ball_info.isLocationKnown())
            flagomni=true;

    printf("KINECT BALL angle: %d radians: %f\n",kinect_ball_info.getRealLocation().angle_.degree(),kinect_ball_info.getRealLocation().radius_);
    printf("OMNI BALL angle: %d radians: %f\n",omni_ball_info.getRealLocation().angle_.degree(),omni_ball_info.getRealLocation().radius_);

    if (flagkinect && flagomni)
    {
        if (kinect_ball_info.getRealLocation().angle_.degree()>-40 && kinect_ball_info.getRealLocation().angle_.degree()<40 && kinect_ball_info.getRealLocation().radius_>=100 && kinect_ball_info.getRealLocation().radius_<550)
        {
            own_ball_=kinect_ball_info;
            printf("LAST KINECT BALL angle: %d radians: %f\n",kinect_ball_info.getRealLocation().angle_.degree(),kinect_ball_info.getRealLocation().radius_);
        }
        else
        {
                own_ball_=omni_ball_info;
            printf("LAST OMNI BALL angle: %d radians: %f\n",omni_ball_info.getRealLocation().angle_.degree(),omni_ball_info.getRealLocation().radius_);

        }
    }
    if (!flagkinect && flagomni)
    {
            own_ball_=omni_ball_info;
            printf("LAST OMNI BALL angle: %d radians: %f\n",omni_ball_info.getRealLocation().angle_.degree(),omni_ball_info.getRealLocation().radius_);
    }
    if (flagkinect && !flagomni)
    {
            own_ball_=kinect_ball_info;
            printf("LAST KINECT BALL angle: %d radians: %f\n",kinect_ball_info.getRealLocation().angle_.degree(),kinect_ball_info.getRealLocation().radius_);
    }
    if (!flagkinect && !flagomni)
    {
            own_ball_.setLocationKnown(false);
            printf("NO BALL !!!!\n");
    }

    /// 如果全向有消息,且有速度,则取全向的速度,否则,取之前的速度
    if (omni_ball_info.isValid()&&omni_ball_info.isVelocityKnown())
    {
        own_ball_.setVelocity(omni_ball_info.getVelocity());
        own_ball_.setVelocityKnown(true);
    }
    else
    {
        own_ball_.setVelocity(velocity);
        own_ball_.setVelocityKnown(is_velocity);
    }
    /// 重新求取速度，则将所有数据清空
    /// 球速需要单独更新，因为足球位置有效，并不代表球速估计就有效，参见球速估计代码

    /// 如果检测到球,默认最终融合的足球采用自己检测到的
    if(own_ball_.isValid() && own_ball_.isLocationKnown())
        ball_info_state_ = SEEBALLBYOWN;
    fuse_ball_ = own_ball_;

    /// 如果不知道球速，且存在被选中的更新足球信息的机器人A，那么可以设置球速为A的球速
    if(!fuse_ball_.isVelocityKnown() && is_valid)
    {
       fuse_ball_.setVelocity(teammates_ball.getVelocity());
       fuse_ball_.setVelocityKnown(teammates_ball.isVelocityKnown());
    }
    /// 如果自己检测不到足球，队友能够感知到足球,足球用队友的
    if(ball_info_state_ == NOTSEEBALL && is_valid)
    {
        ball_info_state_ = SEEBALLBYOTHERS;
        fuse_ball_ = teammates_ball;
    }
}

//! @brief we can calculate the ball velocity according to the least square method
//! @param _ball_info, information of the ball;
//! @param _ball_global_vec, information of the ball;

bool
Ball::evaluateVelocity(std::vector<BallObject> & _ball_info,
                       std::  vector<ros::Time> & _count_time, int & _nums_predict_errors)
{
    /** 默认足球速度信息未知，最终求取到了速度就赋值*/
    int nums_record=_ball_info.size();
    _ball_info[nums_record-1].setVelocity(DPoint(0,0));
    _ball_info[nums_record-1].setVelocityKnown(false);

    /** 如果两者数据不等，时间于球的信息不匹配，足球位置未知，清空，重新求取球速 */
    if((_ball_info.size()!=_count_time.size())||
            !_ball_info[_ball_info.size()-1].isLocationKnown()||
            !_ball_info[_ball_info.size()-1].isValid())
        return true;

    /** 足球于机器人距离过大，认为误差比较大，不估计球速 */
    double distanceb2r=_ball_info[nums_record-1].getRealLocation().radius_;
    if(distanceb2r>600)
        return true;

    /** 只保留nums_record_max帧左右的数据计算球速*/
    const int nums_record_min = 20;
    const int nums_record_max = 80;
    if(nums_record>nums_record_max)
    {
        _ball_info.erase(_ball_info.begin());
        _count_time.erase(_count_time.begin());
    }
    nums_record = _ball_info.size();

    bool ball_vec_known=true;
    DPoint ball_global_vec =  DPoint(0,0);
    /** 记录时间以及位置，当前位置位零点，用于拟合速度*/
    std::vector<double> time_duration;
    std::vector<DPoint> ball_global_pos;
    time_duration.reserve(nums_record_max);
    ball_global_pos.reserve(nums_record_max);
    time_duration.clear();
    ball_global_pos.clear();

    for(int i = 0 ;i < nums_record;i++)
    {
        ros::Duration duration = _count_time[i]-_count_time[nums_record-1];
        double duration_tmp = duration.toSec();
        if(fabs(duration_tmp)<2)
        {
            time_duration.push_back(duration_tmp);
            ball_global_pos.push_back(_ball_info[i].getGlobalLocation());
        }
    }

    nums_record = time_duration.size();
    /** 数据过少，返回false表示不需要清空记录的数据，但是球速没有*/
    if(nums_record < nums_record_min)
    {
        ball_global_vec = DPoint(0,0);
        ball_vec_known =false;
        return false;
    }
    /** 采用最小二乘算法求取x轴、y轴速度*/
    double sum_t(0),sum_tt(0),sum_x(0),sum_y(0),sum_xt(0),sum_yt(0),sum_xx(0),sum_yy(0);
    for(std::size_t i =0 ;i < nums_record ; i++)
    {
        sum_t  += time_duration[i];
        sum_tt += time_duration[i]*time_duration[i];
        sum_x  += ball_global_pos[i].x_;
        sum_y  += ball_global_pos[i].y_;
        sum_xt += time_duration[i]*ball_global_pos[i].x_;
        sum_yt += time_duration[i]*ball_global_pos[i].y_;
        sum_xx +=ball_global_pos[i].x_*ball_global_pos[i].x_;
        sum_yy +=ball_global_pos[i].y_*ball_global_pos[i].y_;
    }
    double detM=nums_record*sum_tt-sum_t*sum_t;
    double p0x(0), p0y(0),tempvx(0),tempvy(0);
    double x_coefd(0),y_coefd(0),x_coefc(0),y_coefc(0),x_stdError(0),y_stdError(0);

    if (fabs(detM) > DBL_EPSILON)
    {
        tempvx=(nums_record*sum_xt-sum_t*sum_x)/detM;
        p0x=(sum_x-tempvx*sum_t)/double(nums_record);

        double sx = tempvx*(sum_xt - sum_x*sum_t/double(nums_record));
        double sy2 = sum_xx-sum_x*sum_x/double(nums_record);
        double sy = sy2 - sx;
        x_coefd = sx / sy2;
        x_coefc = sqrt(x_coefd);
        x_stdError = sqrt(sy / double(nums_record - 2));

        tempvy=(nums_record*sum_yt-sum_t*sum_y)/detM;
        p0y=(sum_y-tempvy*sum_t)/double(nums_record);
        sx = tempvy* ( sum_yt - sum_y * sum_t / double(nums_record) );
        sy2 = sum_yy- sum_y * sum_y / double(nums_record);
        sy = sy2 - sx;
        y_coefd = sx / sy2;
        y_coefc = sqrt(y_coefd);
        y_stdError = sqrt(sy / double(nums_record - 2));
    }
    else
    {
        ball_global_vec = DPoint(0,0);
        ball_vec_known=false;
        return false;
    }
    /** 求取的足球速度 */
    DPoint ball_velocity = DPoint(tempvx,tempvy);
    /** 根据拟合的曲线得到的当前的足球位置，并于实际值相比，求取偏差，如果连续几帧偏差较大，球速重新求取*/
    DPoint ball_predict=DPoint(p0x,p0y);
    double bias=ball_predict.distance(ball_global_pos[nums_record-1]);
    if(bias > BALL_PREDICT_BIAS_CONST)
        _nums_predict_errors++;
    else
        _nums_predict_errors=0;
    int collision_const=5;
    bool is_start_again = false;
    if(_nums_predict_errors >= collision_const)
    {
        is_start_again=true;
        _nums_predict_errors=0;
    }

    /** 求取最近几帧的平均的球速，5帧之前的球速不予考虑，并于当前求取的球速按比例融合*/
    ball_global_vec = ball_velocity;
    DPoint accumulate_pt(0,0);
    double count = 0;
    int count_number = nums_record;
    if(nums_record > 6)
        count_number = 6;
    for(std::size_t i = 1 ;i < count_number ;i++) //从上一帧开始计算
    {
        if(_ball_info[nums_record-i].isVelocityKnown())
        {
            accumulate_pt += _ball_info[nums_record-i-1].getVelocity();
            count++;
        }
    }
    if(count > 0)
    {
        accumulate_pt=1.0/count*accumulate_pt;
        double weight_avgpart=0.3;
        ball_global_vec=accumulate_pt*weight_avgpart+ball_velocity*(1.0-weight_avgpart);
    }
    //! 如果求取的速度太小，且没有连续几帧误差较大，则认为其速度为零
    if(is_start_again)//ball_global_vec.length() < (distanceb2r/20.0+20.0)||
        ball_global_vec=DPoint(0,0);
    //! 连续几帧误差比较大(is_start_again=true)，将其速度设置为未知，并且还要清空数据
    ball_vec_known = !is_start_again;
    _ball_info[nums_record-1].setVelocity(ball_global_vec);
    _ball_info[nums_record-1].setVelocityKnown(ball_vec_known);
    return is_start_again;
}

