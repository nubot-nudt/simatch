#ifndef PASSSTRATEGY_H
#define PASSSTRATEGY_H
#include "nubot/nubot_control/world_model_info.h"
#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/fieldinformation.h>
#include <nubot/nubot_control/plan.h>

namespace nubot {

class PassStrategy
{
public:
    PassStrategy();
    void process();

public:
World_Model_Info * world_model_;
Plan  * m_plan_;

};

}


#endif // PASSSTRATEGY_H
