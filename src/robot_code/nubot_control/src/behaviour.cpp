#include "nubot/nubot_control/behaviour.hpp"
using namespace nubot;

Behaviour::Behaviour()
{
 app_vx_ = 0;
 app_vy_ = 0;
 app_w_  = 0;
 isTurn_ =false;
 last_app_vx_ = 0;
 last_app_vy_ = 0;
 last_app_w_ = 0;
}
Behaviour::~Behaviour()
{
}
