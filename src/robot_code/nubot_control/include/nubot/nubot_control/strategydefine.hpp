#ifndef _NUBOT_STRATEGY_DEFINE_H
#define _NUBOT_STRATEGY_DEFINE_H

#include "nubot/core/core.hpp"

const unsigned char  CTRL_STOP                   =    0 ;
const unsigned char  CTRL_ATTACK                 =    1 ;
const unsigned char  CTRL_GOALKEEP               =    2 ;
const unsigned char  CTRL_DEFENCE                =    3 ;
const unsigned char  CTRL_PASS                   =    4 ;
const unsigned char  CTRL_CATCH                  =    5 ;
const unsigned char  CTRL_PASS_MOVE              =    6 ;
const unsigned char  CTRL_CATCH_MOVE             =    7 ;
const unsigned char  CTRL_CATCH_FOCUS            =    8 ;
const unsigned char  CTRL_MOVET                  =   11 ;
const unsigned char  CTRL_BLOCK		         =   12 ;
const unsigned char  CTRL_ATTENTIONBALL          =   13 ;
const unsigned char  CTRL_PRODEF	         =	14 ;
const unsigned char  CTRL_SEARCHBALL             =	15 ;
const unsigned char  CTRL_SHIFTATK	         =	16 ;
const unsigned char  CTRL_REMOTECTRL             =	17 ;
const unsigned char  CTRL_AROUNDBALL             =	18 ;
const unsigned char  CTRL_ATKCOVER	         =	19 ;
const unsigned char  CTRL_BACKPOS	         =	20 ;
const unsigned char  CTRL_LSATK		         =	21 ;
const unsigned char  CTRL_LSATKCOVER	         =	22 ;
const unsigned char  CTRL_ZONEDEF  	         =	23 ;
const unsigned char  CTRL_TEST	                =       24 ;
const unsigned char  CTRL_IDLE		        =       25 ;
const unsigned char  CTRL_KICKOFF_PRIM_READY    =	26 ;
const unsigned char  CTRL_KICKOFF_SLAVE_READY   =	27 ;
const unsigned char  CTRL_KICKOFF_PRIM		=	28 ;
const unsigned char  CTRL_KICKOFF_SLAVE		=	29 ;
const unsigned char  CTRL_FREEKICK_PRIM_READY   =	30 ;
const unsigned char  CTRL_FREEKICK_SLAVE_READY  =	31 ;
const unsigned char  CTRL_FREEKICK_PRIM		=       32 ;
const unsigned char  CTRL_FREEKICK_SLAVE        =	33 ;
const unsigned char  CTRL_GOALKICK_PRIM_READY   =	34 ;
const unsigned char  CTRL_GOALKICK_SLAVE_READY	=	35 ;
const unsigned char  CTRL_GOALKICK_PRIM	        =	36 ;
const unsigned char  CTRL_GOALKICK_SLAVE	=       37 ;
const unsigned char  CTRL_THROWIN_PRIM_READY	=	38 ;
const unsigned char  CTRL_THROWIN_SLAVE_READY	=	39 ;
const unsigned char  CTRL_THROWIN_PRIM		=	40 ;
const unsigned char  CTRL_THROWIN_SLAVE		=	41 ;
const unsigned char  CTRL_CORNERKICK_PRIM_READY	=	42 ;
const unsigned char  CTRL_CORNERKICK_SLAVE_READY=	43 ;
const unsigned char  CTRL_CORNERKICK_PRIM	=	44 ;
const unsigned char  CTRL_CORNERKICK_SLAVE	=	45 ;
const unsigned char  CTRL_PENALTY_READY		=	46 ;
const unsigned char  CTRL_PENALTY		=	47 ;


//const double          FIELDLENGTH     =  1800;
//const double          FIELDWIDTH      =  1200;
const double          LOCATIONERROR    =  30;
const double          MAXWHELLVELOCITY = 432;
const double          FRICONRATIO      = 0.85;

const double          RADIUS = 30;
const double          GEARREDUCTIONRATIO = 21.3571;
const double          MAXVEL   =    400;
const double          MAXW     =    15;
const double          Max_ObsVision = 10;
const double          OBLE_RADIUS   = 30;
const double          POSITION_LIMIT_VEL =  250;
const double          CHASSISRADIUS =  20.3;   //cm the radius of chassiss
const double          REDUCTIONRATIO = 12.0;  //   reduction gear ratio
const double          WHEELDIAMETER = 12.0;    //cm the diameter of wheel
const double          RATE =  (REDUCTIONRATIO*60.0)/(SINGLEPI_CONSTANT*WHEELDIAMETER); // rate from 4 wheels linear velocity to rpm
const double          LIMITEDRPM  =  12000.0;    // the top limited rpm
const double          LIMITDRIBLLEDIS = 75.0;   // the limit distance to check dribble or not
const double          SCAlEOFOUTFIELD = 500.0;
const double          BALLHOLDINGDIS = 38.0;

enum ACTION_STATE
{
    Stucked,
    Penalty,
    CannotSeeBall,
    SeeNotDribbleBall,
    AtShootSituation,
    PrepareForShoot,
    PathPlan,
    AvoidObs,
    ACProtectBallNearby,
    Positioned,

    CoopKickQuickShoot,
    DynamicShoot,
    TurnForShoot,
    MoveToPass,
    TurnToPass
};


enum BALLSTATE
{
    NOTSEEBALL,
    SEEBALLBYOWN,
    SEEBALLBYOTHERS
};


enum LOCATIONSTATE
{
    INOURFIELD,
    INOPPFIELD,
    INOPPGOALKEEPERZONE,
    INOURPENALTY,
    INOPPPENALTY,
    OUTOFFIELD
};

const double   BOARDWIDTH   =   30.0;
const double   VISIONSCALE	 =	500.0;//500

const double    WIDTHRATIO4FIELD = 1.0;

const double    FIELDLENGTH= 1800.0;//the field length  1800cm
const double    FIELDWIDTH = WIDTHRATIO4FIELD*1200.0;//the field width   1200cm
const double    CORNERSIZE = WIDTHRATIO4FIELD*200.0; // the corner size  200cm
//
const double    GOALWIDTH=WIDTHRATIO4FIELD*200.0;    // the width of the goal

const double    OurPenltyAreaX=WIDTHRATIO4FIELD*150*(FIELDLENGTH/1200.0);
const double    OurPenltyAreaY=WIDTHRATIO4FIELD*150*(FIELDLENGTH/1200.0)+GOALWIDTH/2;
const double    OurGoalKeeperAreaX=WIDTHRATIO4FIELD*50*(FIELDLENGTH/1200.0);
const double    OurGoalKeeperAreaY=WIDTHRATIO4FIELD*50*(FIELDLENGTH/1200.0)+GOALWIDTH/2;
//
const double    OppPenaltyAreaX = WIDTHRATIO4FIELD*150*(FIELDLENGTH/1200.0);
const double    OppPenaltyAreaY = WIDTHRATIO4FIELD*150*(FIELDLENGTH/1200.0)+GOALWIDTH/2;

const double    OppGoalKeeperAreaX=WIDTHRATIO4FIELD*50*(FIELDLENGTH/1200.0);
const double    OppGoalKeeperAreaY=WIDTHRATIO4FIELD*50*(FIELDLENGTH/1200.0)+GOALWIDTH/2;

const double    OBSRECORDNUM = 10.0;

#define deg(a) (SINGLEPI_CONSTANT*(a)/180.0f)

#endif // _NUBOT_STRATEGY_DEFINE_H
