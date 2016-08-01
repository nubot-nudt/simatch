#include "nubot/nubot_control/staticpass.h"

using namespace nubot;

StaticPass::StaticPass()
{
    isPosition_=false;
    m_nCanBeInPenaltyArea=0;
    m_nPassNumber_=-1;
    m_nCatchNumber_=-1;
    ballNumber_=-1;
    ballPos_=DPoint(0,0);
    backFieldPoint_=DPoint(-600,0);
    for(int i=0;i<OUR_TEAM;i++)
    {
        isAllocation_[i]=false;
        targetInit_[i]=DPoint(0,0);
    }
    targetInit_[0]=DPoint(-890,0);                 //静态站位时，守门员基本位置恒定
    target_=DPoint(0,0);                        //为分配的目标点
    m_nPassNumber_=0;
    m_nCatchNumber_=0;
}

StaticPass::~StaticPass()
{}
