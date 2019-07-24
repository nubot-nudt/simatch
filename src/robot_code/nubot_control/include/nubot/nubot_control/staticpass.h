#ifndef STATICPASS_H
#define STATICPASS_H

#include<core.hpp>
#include<nubot/nubot_control/world_model_info.h>
#include<ros/ros.h>

#define PI 3.1415926

const double TAC_DIST_ANTI=330;                      //对方发球时，我方机器人与球之间的距离
const double TAC_DIST_DEF=100;
const double TAC_DIST_DROPBALL=150;                  //Dropball时，我方机器人与球之间的距离
const double TAC_DIST_PASS=100;                      //我方开球时，传球机器人与球之间的距离
const double TAC_DIST_CATCH=235;                     //我方开球时，接球机器人与球之间的距离

const double Ang2Rad=PI/180.0;                  //角度转换为弧度

namespace nubot {

class StaticPass                                 //先考虑能看到球的情况
{
public:
    StaticPass();
    ~StaticPass();

    void    targetInitialize();                     // 目标点初始化
    void    staticReady_();                         //准备站位

    void    OurDefaultReady_();
    void    OppDefaultReady_();
    void    OurPenaltyReady_();
    void    OppPenaltyReady_();
    void    OurkickoffReady_();
    void    OppkickoffReady_();
    void    DropBallReady_();

public:

    World_Model_Info * world_model_;

    bool isPosition_;                            //是否分配完毕无冲突
    int largestNum_;                             //在场编号最大的机器人
    bool isAllocation_[OUR_TEAM];                //该机器人是否分配位置
    int m_nCanBeInPenaltyArea;                   //能在禁区中的机器人个数
    int m_nPassNumber_;                          //传球机器人编号
    int m_nCatchNumber_;                         //接球机器人编号
    int ballNumber_;                             //最后采用球的编号
    int targetNumber_[OUR_TEAM];                 //机器人最后分配的目标点编号
    DPoint targetInit_[OUR_TEAM];                //未分配的初始目标点        0:守门员位置，1:发球机器人位置，2:接球机器人位置，3and4:其他位置
    DPoint target_;                              //该机器人的站位点
    DPoint ballPos_;                             //混合后的球坐标
    DPoint backFieldPoint_;                      //后卫点
};

}
#endif //STATICPASS_H
