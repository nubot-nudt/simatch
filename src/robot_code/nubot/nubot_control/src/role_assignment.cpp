#include "nubot/nubot_control/role_assignment.h"
#include<algorithm>
using namespace nubot;

RoleAssignment::RoleAssignment()
{
    currentRoleTime_    = lastRoleChangeTime_ = ros::Time::now();
}

RoleAssignment::RoleAssignment(World_Model_Info & _world_model)
{
    world_model_ = & _world_model;
    currentRoleTime_    = lastRoleChangeTime_ = ros::Time::now();
}

RoleAssignment::~RoleAssignment()
{}

