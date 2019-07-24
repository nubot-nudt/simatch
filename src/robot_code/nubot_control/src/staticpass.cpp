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

void StaticPass::staticReady_()                    //判断何种站位
{
    switch (world_model_->CoachInfo_.MatchMode)
    {
    case OUR_KICKOFF:
        //OurkickoffReady_();
        OurDefaultReady_();
        break;
    case OPP_KICKOFF:
        //OppkickoffReady_();
        OurDefaultReady_();
        break;
    case OUR_FREEKICK:
        OurDefaultReady_();
        break;
    case OPP_FREEKICK:
        //OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_GOALKICK:
        OurDefaultReady_();
        break;
    case OPP_GOALKICK:
        //OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_CORNERKICK:
        OurDefaultReady_();
        break;
    case OPP_CORNERKICK:
        //OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_THROWIN:
        OurDefaultReady_();
        break;
    case OPP_THROWIN:
        //OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_PENALTY:
        //OurPenaltyReady_();
        OurDefaultReady_();
        break;
    case OPP_PENALTY:
        //OppPenaltyReady_();
        OurDefaultReady_();
        break;
    case DROPBALL:
        //DropBallReady_();
        OurDefaultReady_();
        break;
    default:
        break;
    }
}

void StaticPass::OurDefaultReady_()                   //我方发球默认的站位
{
}

void StaticPass::OppDefaultReady_()               //对方发球默认站位
{

}

void StaticPass::OurPenaltyReady_()
{

}

void StaticPass::OppPenaltyReady_()               //对方penalty发球
{

}

void StaticPass::OurkickoffReady_()                   //我方kickoff发球站位
{

}

void StaticPass::OppkickoffReady_()               //对方kickoff发球站位
{

}

void StaticPass::DropBallReady_()                  //dropball站位
{

}

void StaticPass::targetInitialize()
{
    // 给所有站位点赋值，防止后面也没有赋值导致出错
    targetInit_[0]=DPoint(-890,0);                  //静态站位时，守门员基本位置恒定
    targetInit_[1]=DPoint(-500,200);
    targetInit_[2]=DPoint(-500,-200);
    targetInit_[3]=DPoint(-200,100);
    targetInit_[4]=DPoint(-200,-100);
}
