#include "nubot/nubot_control/activerole.h"

using namespace nubot;

ActiveRole::ActiveRole()
{
    stuckflg_ = false;
    pass_lock_ = 0;
    stucktime_ = 0;
    NeedEvaluat = false;
    dynamic_shoot_state_ = false;
    dynamic_shoot_count_ = 0;
    quick_shoot_state_ = false;
    quick_shoot_count_ = 0;
    currentstate_ = CanNotSeeBall;
    catch_in_ourfeild_ =false;
    kick_enable_ =false;
}
ActiveRole:: ~ActiveRole()
{
}
