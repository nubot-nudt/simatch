#ifndef STRATEGY_H
#define STRATEGY_H
#include <cmath>
#include"core.hpp"
//#include "nubot/nubot_control/goaliestrategy.h"
#include "nubot/nubot_control/role_assignment.h"
#include "nubot/nubot_control/world_model_info.h"
#include "nubot/nubot_control/goaliestrategy.h"
#include "nubot/nubot_control/plan.h"
#include "nubot/nubot_control/activerole.h"
#include "nubot/nubot_control/midfieldrole.h"
#include "nubot/nubot_control/passiverole.h"
#include "nubot/nubot_control/assistrole.h"
namespace nubot{

class Strategy
{

public:
    Strategy();
    Strategy(World_Model_Info & _world_model, Plan & _plan);
    ~Strategy();
   void selectRole();
   void selectAction();
   void process();
   bool passStrategy();
public:
    RoleAssignment    RoleAssignment_;
    World_Model_Info * world_model_;
    int selected_role_;
    int selected_action_;
    Plan * m_plan_;
    ActiveRole        ActiveRole_;
    AssistRole        AssistRole_;
    PassiveRole       PassiveRole_;
    MidfieldRole      MidfieldRole_;
    GoalieStrategy    goalie_strategy_;
    bool  auto_competition;
};

}
#endif // STRATEGY_H
