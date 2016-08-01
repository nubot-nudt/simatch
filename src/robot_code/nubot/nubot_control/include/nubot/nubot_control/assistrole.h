#ifndef ASSISTROLE_H
#define ASSISTROLE_H

#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/fieldinformation.h>
#include <nubot/nubot_control/plan.h>

namespace nubot {

class AssistRole
{
public:
      AssistRole();
     ~AssistRole();

      void process();     //根据当前状态选择需要执行的动作,现在只考虑了站位
      void assistCalculate();

      bool isBestPass(DPoint world_pt);       //是否在最佳的接球区域
      bool isObsActiver(DPoint candidate_point);     //是否阻碍到主攻射门
      bool isNullInTrap(const DPoint & robot_pos, const std::vector<DPoint> & obs_pts);          //周围是否有障碍物

public:
      World_Model_Info * world_model_;
      Plan *plan_;

      bool isInOurfeild_;                     //用于站位决策的两个因素
      bool IsOurDribble_;

      //DPoint best_pass_[4];
};

}
#endif // ASSISTROLE_H
