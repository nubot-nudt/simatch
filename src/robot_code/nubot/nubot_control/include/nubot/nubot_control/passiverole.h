#ifndef PASSIVEROLE_H
#define PASSIVEROLE_H

#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/fieldinformation.h>
#include <nubot/nubot_control/plan.h>

namespace nubot {

class PassiveRole
{
public:
      PassiveRole();
     ~PassiveRole();

      void process();
      void passiveCalculate();                                //新的站位点计算
      void findPointOut(Angle ang_goal2ball);                  //找到这个方向上出禁区的点
      void awayFromActive();

public:
      World_Model_Info * world_model_;
      Plan *plan_;

      bool isInOurfeild_;                     //用于站位决策的两个因素
      bool IsOurDribble_;

      DPoint defence_pos_;
      Angle  defence_ori_;

};

}
#endif // PASSIVEROLE_H
