#include <iostream>
#include <fstream>
#include "nubot/nubot_control/goaliestrategy.h"

nubot::GoalieStrategy::GoalieStrategy()
{
  robot_info_.pos.x = -850;
  robot_info_.pos.y = 0;
  robot_info_.heading.theta = 0;
  robot_info_.vtrans.x = 0;
  robot_info_.vtrans.y = 0;
  ball_info_2d_ .pos_known = false;
  ball_info_3d_.pos_known_3d = false;
  ball_info_3d_.pos_known_2d = false;
  state_ = Move2Origin;
}

nubot::ParabolaFitter3D::ParabolaFitter3D()
:n_(0),fly_flag_(0),data_pointer_(-1)
{
}
