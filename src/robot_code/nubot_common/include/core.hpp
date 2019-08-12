#ifndef __NUBOT_CORE_HPP__
#define __NUBOT_CORE_HPP__

#include <fstream>
#include <iostream>
#include <float.h>
#include <string>
//#include <opencv2/opencv.hpp>
#include "Circle.hpp"
#include "Angle.hpp"
#include "DPoint.hpp"
#include "PPoint.hpp"
#include "Line.hpp"

/// 关于当前模式的选择（正常比赛，赛后点球，仿真）
//#define MATCH
//#define END_GAME_PENALTY
#define SIMULATION
//#define using_openmp

/// 该标志为表示升降机构的位置
#define  PASS    1
#define  SHOOT   -1

/// 对方发球时，需要等待7秒之后才能动
#define WAIT_SECS           7
/// 如果是IN-GAME-PENALTY, 需要等待10秒之后才能动
#define PENALTY_WAIT_SECS   10

#if defined(SIMULATION)
const double MAXVEL                 = 500;
const double MAXW                   = 10;
const double WIDTH_RATIO            = 1;
const double LENGTH_RATIO           = 1;
const int FIELD_CENTER_RADIUS       = 200;
#elif defined(MATCH)
const double MAXVEL                 = 400;
const double MAXW                   = 8;
const double WIDTH_RATIO            = 14.0/12.0;
const double LENGTH_RATIO           = 22.0/18.0;
const int FIELD_CENTER_RADIUS       = 200;
#else
const double MAXVEL                 = 400;
const double MAXW                   = 10;
const double WIDTH_RATIO            = 8.0/12.0;
const double LENGTH_RATIO           = 1;
const int FIELD_CENTER_RADIUS       = 150;
#endif

/// 数据没有更新的阈值，比如通信过程中时间大于300ms为更新数据，默认为失效
const int MAX_OBSNUMBER_CONST = 10;
const int NOT_DATAUPDATE    = 1200;
const int OUR_TEAM          = 5 ;
const int OPP_TEAM          = 7 ;
const int ROLENUM           = 7;

/// 一些常用的场地参数
#if defined(SIMULATION)
const int FIELD_LENGTH  = 2200;
const int FIELD_WIDTH   = 1400;
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH);
#elif defined(MATCH)
const int FIELD_LENGTH  = 2200;
const int FIELD_WIDTH   = 1400;
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH);
#else
const int FIELD_LENGTH  = 1800;
const int FIELD_WIDTH   = 800;
/// 对角线长度
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH);
#endif

/// 场地缩放单位长度
#if defined(MATCH)
#define  WIDTH  748/2200
#define  HEIGHT 476/1400
#else
#define  WIDTH 748/2200
#define  HEIGHT 499/1400
#endif

/// 场地上的几条水平和垂直线,，其他的信息都可以通过场地信息得到
#if defined(SIMULATION)
const int FIELD_XLINE1 = 1100;
const int FIELD_XLINE2 = 1025;
const int FIELD_XLINE3 = 875;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -875;
const int FIELD_XLINE6 = -1025;
const int FIELD_XLINE7 = -1100;
#elif defined(MATCH)   //modify according to the new field
const int FIELD_XLINE1 = 1100;
const int FIELD_XLINE2 = 1025;
const int FIELD_XLINE3 = 875;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -875;
const int FIELD_XLINE6 = -1025;
const int FIELD_XLINE7 = -1100;
#else
const int FIELD_XLINE1 = 900;
const int FIELD_XLINE2 = 825;
const int FIELD_XLINE3 = 675;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -675;
const int FIELD_XLINE6 = -825;
const int FIELD_XLINE7 = -900;
#endif

#if defined(SIMULATION)
const int FIELD_YLINE1 =  700;
const int FIELD_YLINE2 =  345;
const int FIELD_YLINE3 =  195;
const int FIELD_YLINE4 =  -195;
const int FIELD_YLINE5 =  -345;
const int FIELD_YLINE6 =  -700;
#elif defined(MATCH)   //modify according to the new field
const int FIELD_YLINE1 =  700;
const int FIELD_YLINE2 =  345;
const int FIELD_YLINE3 =  195;
const int FIELD_YLINE4 =  -195;
const int FIELD_YLINE5 =  -345;
const int FIELD_YLINE6 =  -700;
#else
const int FIELD_YLINE1 = 400;
const int FIELD_YLINE2 = 217;
const int FIELD_YLINE3 = 117;
const int FIELD_YLINE4 = -117;
const int FIELD_YLINE5 = -217;
const int FIELD_YLINE6 = -400;
#endif

const int FIELD_POST_RADIUS = 80;
const int LOCATIONERROR     = 30;
const int ANGLEERROR        = 5;
/// 如果看到的对方机器人距离小于它，则认为是同一个障碍物
const double COMBINE_OPP_DIS = 70;

/// 场地边界
enum Border{LEFTBORDER  = 0,
            RIGHTBORDER = 1,
            UPBORDER    = 2,
            DOWNBORDER  = 3};

/// 球门某特殊点位置
enum GoalLocation{GOAL_UPPER     = 0,
                  GOAL_MIDUPPER  = 1,
                  GOAL_MIDDLE    = 2,
                  GOAL_MIDLOWER  = 3,
                  GOAL_LOWER     = 4};

/// 机器人角色定义（后面两个从没看到）
enum Roles{GOALIE       = 0,
           ACTIVE       = 1,
           PASSIVE      = 2,
           MIDFIELD     = 3,
           ASSISTANT    = 4,
           ACIDPASSIVE  = 5,
           GAZER        = 6,
           BLOCK        = 7,
           NOROLE       = 8,
           CATCHOFPASS  = 9,
           PASSOFPASS   =10};

/// 比赛指令
enum MatchMode {
                 STOPROBOT      = 0,
                 OUR_KICKOFF    = 1,
                 OPP_KICKOFF    = 2,
                 OUR_THROWIN    = 3,
                 OPP_THROWIN    = 4,
                 OUR_PENALTY    = 5,
                 OPP_PENALTY    = 6,
                 OUR_GOALKICK   = 7 ,
                 OPP_GOALKICK   = 8,
                 OUR_CORNERKICK = 9,
                 OPP_CORNERKICK = 10,
                 OUR_FREEKICK   = 11,
                 OPP_FREEKICK   = 12,
                 DROPBALL       = 13,
                 STARTROBOT     = 15,
                 PARKINGROBOT   = 25,
                 TEST           = 27
               };

/// 用于动态调整决策（偏进攻or防守），目前也没用了
enum StrategyTyples {  STRATEGY_ATTACK  = 0,
                       STRATEGY_DEFEND  = 1,
                       STRATEGY_BALANCE = 2,
                       STRATEGY_AUTO    = 4,
                       NOSTRATEG        = 5
};

/// 测试项目
enum TestMode
{
    Test_Stop           = 0,
    Move_NoBall_NoAvoid = 10,
    Move_NoBall_Avoid   = 11,
    Move_Ball_NoAvoid   = 12,
    Move_Ball_Avoid     = 13,
    Pass_Ball           = 2,
    Catch_Ball          = 3,
    Shoot_Ball          = 4,
    Location_test       = 5,
    Circle_test         = 6
};


/// 机器人动作
enum Actions
{
    Stucked           =0,
    Penalty           =1,
    CanNotSeeBall     =2,
    SeeNotDribbleBall =3,
    TurnForShoot      =4,
    TurnForShoot_Robot=5,
    AtShootSituation  =6,
    TurnToPass        =7,
    TurnToPass_Robot  =8,
    StaticPass        =9,
    AvoidObs          =10,
    Catch_Positioned  =11,
    Positioned        =12,
    Positioned_Static =13,
    KickCoop          =14,
    KickCoop_turn     =15,
    CatchBall         =16,
    CatchBall_slow    =17,
    CircleTest        =18,
    MoveWithBall      =19,
    TeleopJoy         =20,
    No_Action         =21,
};

/// 简约的障碍物信息结构体
struct obs_info
{
    nubot::PPoint polar_pt;
    nubot::DPoint world_pt;
    double HRZ[4];
    double HRH[4*4];
};



#endif 
