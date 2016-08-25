#ifndef __NUBOT_CORE_HPP__
#define __NUBOT_CORE_HPP__


#include "Circle.hpp"
#include "Angle.hpp"
#include "DPoint.hpp"
#include "PPoint.hpp"
#include "Line.hpp"

#define  SIMULATION
//#define  NET_TYPE "eth0"

//#define using_openmp

#ifdef  SIMULATION
const double WIDTHRATIO4FIELD = 1;
const double WIDTH_RATIO= 1 ;
const double ConstDribbleDisFirst  = 50;
const double ConstDribbleDisSecond = 40;
#else
const double WIDTHRATIO4FIELD = 8.0/12.0 ;
const double WIDTH_RATIO= 8.0/12.0 ;
const double ConstDribbleDisFirst  = 40;
const double ConstDribbleDisSecond = 35;
#endif
//using namespace nubot;
const int MAX_OBSNUMBER_CONST = 10;
const int NOT_DATAUPDATE = 300; //数据没有更新的阈值，比如通信过程中时间大于300ms为更新数据，默认为失效
const int OUR_TEAM = 5 ;        //自己机器人个数
const int OPP_TEAM = 7 ;        //对方的机器人个数
const int ROLENUM = 11;

/** 一些常用的场地参数 */
const int FIELD_LENGTH= 1800;    //场地参数，长度
const int FIELD_WIDTH = 800;     //场地参度，长度
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH); //对角线长度

/** 场地上的几条水平和垂直线,，其他的信息都可以通过场地信息得到*/
const int FIELD_XLINE1 = 900;
const int FIELD_XLINE2 = 825;
const int FIELD_XLINE3 = 675;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -675;
const int FIELD_XLINE6 = -825;
const int FIELD_XLINE7 = -900;
#ifdef SIMULATION
const int FIELD_YLINE1 =  600;
const int FIELD_YLINE2 =  325;
const int FIELD_YLINE3 =  175;
const int FIELD_YLINE4 =  -175;
const int FIELD_YLINE5 =  -325;
const int FIELD_YLINE6 =  -600;
#else
const int FIELD_YLINE1 = 400;  // 600
const int FIELD_YLINE2 = 217;  // 325
const int FIELD_YLINE3 = 117;  // 175
const int FIELD_YLINE4 = -117; // -175
const int FIELD_YLINE5 = -217; // -325
const int FIELD_YLINE6 = -400; // -600
#endif

const int FIELD_CENTER_RADIUS = 150;
const int FIELD_POST_RADIUS = 0;
const int LOCATIONERROR = 30;

/** 场地边界 **/
enum Border{LEFTBORDER = 0,
            RIGHTBORDER = 1,
            UPBORDER = 2,
            DOWNBORDER = 3};

/** 比赛模式的一些定义，*/
enum Roles{GOALIE = 0 ,
           ACTIVE = 1,
           PASSIVE = 2,
           MIDFIELD = 3,
           ASSISTANT = 4,
           ACIDPASSIVE = 5,
           GAZER = 6 ,
           BLOCK = 7,
           NOROLE = 8,
           CATCHOFPASS = 9,
           PASSOFPASS =10};

enum MatchMode {
                 STOPROBOT  =  0,
                 OUR_KICKOFF = 1,
                 OPP_KICKOFF = 2,
                 OUR_THROWIN = 3,
                 OPP_THROWIN = 4,
                 OUR_PENALTY = 5,
                 OPP_PENALTY = 6,
                 OUR_GOALKICK = 7 ,
                 OPP_GOALKICK = 8,
                 OUR_CORNERKICK = 9,
                 OPP_CORNERKICK = 10,
                 OUR_FREEKICK = 11,
                 OPP_FREEKICK = 12,
                 DROPBALL     = 13,
                 STARTROBOT   = 15,
                 PARKINGROBOT = 25,
                 TEST = 27
               };

enum StrategyTyples {  STRATEGY_ATTACK = 0,
                       STRATEGY_DEFEND = 1,
                       STRATEGY_BALANCE =2,
                       STRATEGY_AUTO = 4,
                       NOSTRATEG =5
};

enum Actions
{
    Stucked =0,
    Penalty =1,
    CanNotSeeBall =2,
    SeeNotDribbleBall =3,
    TurnForShoot =4, // Active
    AtShootSituation =5,
    TurnToPass=6,   //Dynamic pass
    StaticPass =7,
    AvoidObs=8,
    Catch_Positioned = 9,   //catch position
    Positioned =10,          //move to position
    Positioned_Static  =11, //static
    No_Action  =12,
};

struct obs_info
{
    nubot::PPoint polar_pt;
    nubot::DPoint world_pt;
    double HRZ[4];
    double HRH[4*4];
};

enum TestMode
{
    Test_Stop =0,
    Move_NoBall_NoAvoid = 10,
    Move_NoBall_Avoid   = 11,
    Move_Ball_NoAvoid   = 12,
    Move_Ball_Avoid     = 13,
    Pass_Ball  =2,
    Catch_Ball =3,
    Shoot_Ball =4,
    Location_test =5,
    Circle_test =6
};

#endif 
