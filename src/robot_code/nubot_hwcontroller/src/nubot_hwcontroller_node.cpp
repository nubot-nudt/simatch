/// C与C++混合编程要在CPP文件中加extern “C”关键字，否则链接会出错
#include "nubot_hwcontroller/nubot_hwcontroller_node.h"
/// CAN模块位置
#define CAN_SLOT_POSITION 4
#define WHEELS 3
#define USE_BRUSH_MOTOR     0
#define DEFAULT_PRIO    (60)
#define MAX_SAFE_STACK (8*1024)
#define _USE_MATH_DEFINES
#define USE_CURSE 0
#define DribbleDebug 0
#define RADIUS 30.0f
/// 继电器导通到IGBT导通控制时间间隔，共10ms
#define  Delay2IGBT     65
#ifndef min
#define min(x, y) (((x) > (y)) ? (y) : (x))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif
#define deg(a) (PI*(a)/180.0f)
#define Limmit(Vmin,v,Vmax)			(min(max((v),(Vmin)),(Vmax)))
/// 用了比较多的全局变量
using namespace boost::posix_time;
using namespace boost::filesystem;

bool  cmac_enable=false;
const char* msg_state[]={"TryCatch","Firm","Unstable","Stuck"};
const double WHEEL_DISTANCE=23.28;
const double MOTOR_GEAR_RATIO=10.0;
const double WHEEL_DIAMETER=12.5;
const double VEL_TO_RPM_RATIO=MOTOR_GEAR_RATIO*60.0/(M_PI*WHEEL_DIAMETER);
const double LIMITEDRPM=5500;
static std::string robot_name ;
int zero3[3]={0,0,0};
int zero2[2]={0,0};

/// 持球状态
typedef enum {TryCatch=0,Firm,Unstable,Stuck} BallStates;

/// \brief 这里利用list来初始化变量,但实际上有的新加的变量并未出现在list中,存在风险
Nubot_HWController::Nubot_HWController(int argc,char** argv)
    :n(),number(0),motor_left(motor_speed[0]),motor_right(motor_speed[1]),motor_up(0),
      Vx(0),Vy(0),w(0),P(7.0),I(0),D(2.5),BallSensor_IsHolding(false),
      BallHandleEnable(0),ShootEnable(false),shootcnt(0),ShootPos(0),RodPos(0),ShootPower(0),ShootDone(true),
      PowerState(true),PowerState_Last(true),
      acc_state(0),wacc_state(0),
      RotationMode(0),
      HoldingCnt(0),UnholdingCnt(0),
      move_action_(21),rotate_action_(21),
      LeverPos_SetPoint(0),FFRatio_Set(1)
{
//    ROS_INFO("initialize hw_node process");
    robot_name = argv[1];
    std::string num = robot_name.substr(robot_name.size()-1);
    ROS_FATAL("robot_name:%s",robot_name.c_str());
    n = ros::NodeHandle(robot_name);
    motor_cmd_pub_ = n.advertise<nubot_common::VelCmd>("nubotcontrol/velcmd",1);
    /// 接受底盘速度指令
    actioncmd_sub_ = n.subscribe("nubotcontrol/actioncmd",1,&Nubot_HWController::SetAction,this);
    /// 建立周期回调函数
    timer1=n.createTimer(ros::Rate(200),&Nubot_HWController::Timer1_Process,this);
}

Nubot_HWController::~Nubot_HWController()
{

//    Ballhandle_Enable(false);
}

/// \brief 定时器，关系整个底层控制，控制频率500hz
/// BaseController负责底层电机控制
/// DribbleController负责抓球电机控制
/// ShooterController负责升降机构控制
void Nubot_HWController::Timer1_Process(const ros::TimerEvent &e)
{
    std::string num = robot_name.substr(robot_name.size()-1);
//    std::cout<<"timer  "<<num<<std::endl;
    BaseController();
}



/// \brief 十分重要的底层控制，得到Vx，Vy，w后解算到各个电机，并读取当前电机的实际转速作为里程计融合
void Nubot_HWController::BaseController()
{
    std::string num = robot_name.substr(robot_name.size()-1);
//    std::cout<<"received action cmd"<<num<<std::endl;
    /// 由上层得到的动作，目标，以及当前位置等得到机器人体坐标系下的速度
    calculateSpeed();
    nubot_common::VelCmd  vel_cmd;
//    Vx = 0;Vy=0;w = 0;
    vel_cmd.Vx = Vx;
    vel_cmd.Vy = Vy;
    vel_cmd.w  = w;
    if(move_action_==No_Action)
    {
        vel_cmd.Vy = 0;
        vel_cmd.Vx = 0;
    }
    if(rotate_action_==No_Action)
        vel_cmd.w = 0;
//    std::cout<<"num "<<num<<"  vx::"<<Vx<<"  vy:: "<<Vy<<"  w::"<<w<<std::endl;
    motor_cmd_pub_.publish(vel_cmd);
}
/// \brief 上层nubot_control发布的动作topic的回调函数，在自主运行时，订阅的是动作
void Nubot_HWController::SetAction(const nubot_common::ActionCmd::ConstPtr& cmd)
{
    std::string num = robot_name.substr(robot_name.size()-1);
    /// 动作，当前线速度，角速度，目标位置，目标速度，目标角度，最大线速度，最大角速度
    move_action_=cmd->move_action;
    rotate_action_=cmd->rotate_acton;
    rotate_mode_=cmd->rotate_mode;
    target_=nubot::DPoint2s(cmd->target.x,cmd->target.y);
    target_vel_=nubot::DPoint2s(cmd->target_vel.x,cmd->target_vel.y);

    target_ori_=cmd->target_ori;

    robot_vel_=nubot::DPoint2s(cmd->robot_vel.x,cmd->robot_vel.y);
    robot_w_=cmd->robot_w;
    maxvel_=cmd->maxvel;
    maxw_=cmd->maxw;
////    /// 带球及射门消息
    BallHandleEnable=cmd->handle_enable;
    ShootPos=cmd->shootPos;

    /// 当前一次射门结束时，赋予新的射门力量
    if(ShootDone)
        ShootPower=cmd->strength;
    /// 如果是带球状态，最大线速度进一步限制
    if(BallSensor_IsHolding && maxvel_>300)
        maxvel_=300;
    /// 如果不是抓球动作，最大角速度也进一步限制
    if(rotate_action_!=Catch_Positioned&&rotate_action_!=CatchBall&&rotate_action_!=CatchBall_slow&&rotate_action_!=TeleopJoy)
        maxw_=3;
    /// 根据动作可以灵活的设置不同动作的各种限制条件，明显优于以前传速度的形式
    robot_pos_ = nubot::DPoint2s(cmd->robot_pos.x,cmd->robot_pos.y);
    robot_ori_ = cmd->robot_ori;
    //更新topic时间
    TimeFromHighLevel_ = ros::Time::now();
}

/// \brief 根据上层节点的信息计算当前机器人的速度
void Nubot_HWController::calculateSpeed()
{
    //adjustPD(1,0);
    rotate2AbsOrienation();
    move2target();
//    if(rotate_action_==No_Action && move_action_==No_Action)
//    {
//        Vx=Vy=0;
//        w=0;
//    }
}

/// \brief 基本的PD控制器
float Nubot_HWController::basicPDControl(float pgain,float dgain, float err,float err1, float maxval)
{
    float _e1 = err1;
    float kp =  pgain;
    float kd=  dgain;
    float _e=err;
    float retval = 0;
    retval  = kp *_e +kd*(_e -_e1) ;
    if(fabs(retval) > maxval)
    {
        if(retval>0) retval= maxval;
        else    retval=-maxval;
    }
    return retval;
}

/// \brief 动态调节PD参数,pval参考P，dval参考D，weight可调的平动与转动权重
void Nubot_HWController::adjustPD(float pval, float dval,float weight)
{
//    /// 目标点距离误差
//    float pos_e  = robot_pos_.distance(target_);
//    /// 目标点角度误差
//    float theta_e = fabs(target_ori_-robot_ori_.radian_);
//    /// 权重及归一化处理
//    pos_e=pos_e*weight/900;
//    theta_e=theta_e*(1-weight)/(M_PI);

//    /// 求得新的PD参数
//    p_move_=pval/(pos_e+1);
//    d_move_=dval/(pos_e+1);
//    p_rotation_=pval;
//    d_rotation_=dval;
}

/// \brief 根据目标点计算当前的Vx，Vy
void Nubot_HWController::move2target()
{
    float _pos_e = robot_pos_.distance(target_);
    nubot::DPoint2s relposoftarget =  target_  - robot_pos_;
    float tar_theta = relposoftarget.angle().radian_;
    static float _pos_e1 = 0;
    float speed  = 0;
    p_move_ = 5; d_move_ = 0;
    speed = basicPDControl(p_move_,d_move_,_pos_e,_pos_e1,maxvel_);
    Vx =  speed*cos(tar_theta - robot_ori_.radian_) + target_vel_.x_;
    Vy =  speed*sin(tar_theta - robot_ori_.radian_) + target_vel_.y_;

    double v=sqrt(Vx*Vx+Vy*Vy);
    if(v>maxvel_)
    {
        Vx=Vx*maxvel_/v;
        Vy=Vy*maxvel_/v;
    }
    _pos_e1  = _pos_e;
}

/// \brief 根据目标角度计算当前的w
void Nubot_HWController::rotate2AbsOrienation()
{
    float theta_e = target_ori_-robot_ori_.radian_;
    static float theta_e1 =  0;
    // rotate_mode: 0-根据小角度选择旋转方向，1-顺时针方向，-1-逆时针方向
    if(rotate_mode_==-1)
    {
        if(theta_e>0)
            theta_e = theta_e-2*SINGLEPI_CONSTANT;
    }
    else if(rotate_mode_==1)
    {
        if(theta_e<0)
            theta_e = theta_e+2*SINGLEPI_CONSTANT;
    }
    else
    {
        while(theta_e > SINGLEPI_CONSTANT) theta_e = theta_e-2*SINGLEPI_CONSTANT;
        while(theta_e <= -SINGLEPI_CONSTANT) theta_e = theta_e+2*SINGLEPI_CONSTANT;
    }
    p_rotation_=2; d_rotation_=0;
    w = basicPDControl(p_rotation_,d_rotation_,theta_e,theta_e1,maxw_);
    theta_e1 = theta_e ;
}

/// \brief 采用极坐标控制，只产生向前的平移和转动，模拟差动控制避免丢球
void Nubot_HWController::movewithball()
{
//    nubot::DPoint2s robot2target=target_-robot_pos_;
//    float beta=robot2target.angle().radian_-target_ori_;
//    while(beta > SINGLEPI_CONSTANT) beta = beta-2*SINGLEPI_CONSTANT;
//    while(beta <= -SINGLEPI_CONSTANT) beta = beta+2*SINGLEPI_CONSTANT;

//    float alpha=robot2target.angle().radian_-robot_ori_.radian_;
//    while(alpha > SINGLEPI_CONSTANT) alpha = alpha-2*SINGLEPI_CONSTANT;
//    while(alpha <= -SINGLEPI_CONSTANT) alpha = alpha+2*SINGLEPI_CONSTANT;

//    float p=robot_pos_.distance(target_);

//    float k1=1.5,k2=1,k3=1;

//    Vx= k1*p*cos(alpha);
//    Vy= 0;
//    w = k2*alpha+k1*(sin(alpha)*cos(alpha)/alpha)*(alpha+k3*beta);

//    if(Vx>maxvel_)
//        Vx=maxvel_;

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"nubot_hwcontroller_node");
    Nubot_HWController controller(argc,argv);
    ros::spin();
    return 0;
}
