#ifndef STRATEGYINFO_H
#define STRATEGYINFO_H
#include "nubot/core/core.hpp"

namespace nubot {

struct PassCommands //is_pass = true,准备接球，每个机器人都收到，就可以判断状态了
{
    int  passrobot_id;
    int  catchrobot_id;
    bool is_pass;
    bool is_passout;
    DPoint pass_pt;
    DPoint catch_pt;
};

class StrategyInfo
{
public:
    StrategyInfo();
    double from_nubotcontrol_time_; //!从nubot_control更新的topic时间；
    double from_otherrobots_time_;
    PassCommands pass_cmd_;
    PassCommands receive_pass_;
    void update();
};

}

#endif // STRATEGYINFO_H
