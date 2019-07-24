#if 0

#include <ros/ros.h>
// #include "nubot/simulation_interface/multicast.h"
#include <nubot_common/CoachInfo.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot/core/core.hpp>
#include <nubot/world_model/teammatesinfo.h>
#include <nubot/world_model/robot.h>
#include <nubot/rtdb/rtdb_api.h>
#include <nubot/rtdb/rtdb_user.h>
#define MAX_BOT_NUM 5

using namespace nubot;
nubot_common::CoachInfo coach_msg;
MessageFromCoach coach2robot;
Teammatesinfo_sim teammatesinfo_sim;

void DB_get_coach_info(void)
{
    DB_get(0,MESSAGEFROMCOACHINFO,&coach2robot);    //0  暂时代表coach agent
    coach_msg.Head = coach2robot.Head;
    coach_msg.MatchMode = coach2robot.MatchMode;
    coach_msg.MatchType = coach2robot.MatchType;
    ROS_INFO("match_mode:%d match_type:%d\n",coach2robot.MatchMode, coach2robot.MatchType);
}

void receive_worldmodel_info_and_DB_put(const nubot_common::WorldModelInfo & _world_msg)
{
    /** 获取机器人信息 */
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        Robot & robot_info = teammatesinfo_sim.info_sim[i].robot_info_;
        DPoint location(_world_msg.robotinfo[i].pos.x,
                        _world_msg.robotinfo[i].pos.y);
        DPoint vel(_world_msg.robotinfo[i].vtrans.x,
                   _world_msg.robotinfo[i].vtrans.y);
        DPoint target(_world_msg.robotinfo[i].target.x,
                      _world_msg.robotinfo[i].target.y);
        Angle angle(_world_msg.robotinfo[i].heading.theta, true);

        robot_info.setID(_world_msg.robotinfo[i].AgentID);
        robot_info.setLocation(location);
        robot_info.setHead(angle);
        robot_info.setVelocity(vel);
        robot_info.setStuck(_world_msg.robotinfo[i].isstuck);
        robot_info.setKick(_world_msg.robotinfo[i].iskick);
        robot_info.setValid(_world_msg.robotinfo[i].isvalid);
        robot_info.setW(_world_msg.robotinfo[i].vrot);
        robot_info.setCurrentRole(_world_msg.robotinfo[i].current_role);
        // robot_info.setCurrentAction(_world_msg.robotinfo[i].current_action);
        robot_info.setRolePreserveTime(_world_msg.robotinfo[i].role_time);
        robot_info.setTarget(target);
        robot_info.setDribbleState(_world_msg.robotinfo[i].isdribble);
    }

    /** 获取球的信息 */
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        BallObject & ball_info = teammatesinfo_sim.info_sim[i].ball_info_;

        DPoint loc(_world_msg.ballinfo[i].pos.x,
                          _world_msg.ballinfo[i].pos.y);
        DPoint vel(_world_msg.ballinfo[i].velocity.x,
                          _world_msg.ballinfo[i].velocity.y);
        Angle r_ang(_world_msg.ballinfo[i].real_pos.angle, true);
        double r_radius = _world_msg.ballinfo[i].real_pos.radius;
        PPoint r_loc(r_ang, r_radius);

        ball_info.setGlobalLocation(loc);
        ball_info.setRealLocation(r_loc);
        ball_info.setVelocity(vel);
        ball_info.setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        ball_info.setLocationKnown(_world_msg.ballinfo[i].pos_known);
    }

    /**  获取传球信息*/
    for(int i = 0 ; i < OUR_TEAM; i++)
    {
        PassCommands & pass_command = teammatesinfo_sim.info_sim[i].pass_cmds_;
        // 初始化
        pass_command.catchrobot_id = -1;
        pass_command.passrobot_id = -1;
        pass_command.is_dynamic_pass  = false;
        pass_command.is_static_pass  = false;
        pass_command.is_passout = false;
        pass_command.isvalid = false;

        if(_world_msg.pass_cmd.is_valid && (_world_msg.pass_cmd.is_static_pass ||_world_msg.pass_cmd.is_dynamic_pass||_world_msg.pass_cmd.is_passout))
        {
            pass_command.catchrobot_id      =   _world_msg.pass_cmd.catch_id;
            pass_command.passrobot_id       =   _world_msg.pass_cmd.pass_id ;
            pass_command.is_dynamic_pass    =   _world_msg.pass_cmd.is_dynamic_pass;
            pass_command.is_static_pass     =   _world_msg.pass_cmd.is_static_pass;
            pass_command.is_passout         =   _world_msg.pass_cmd.is_passout;
            pass_command.isvalid            =   _world_msg.pass_cmd.is_valid;
            pass_command.pass_pt.x_         =   _world_msg.pass_cmd.pass_pt.x;
            pass_command.pass_pt.y_         =   _world_msg.pass_cmd.pass_pt.y;
            pass_command.catch_pt.x_        =   _world_msg.pass_cmd.catch_pt.x;
            pass_command.catch_pt.y_        =   _world_msg.pass_cmd.catch_pt.y;
        }
    }

    // 将所有队友信息发给Coach
    if(DB_put(TEAMMATESINFO_SIM, &teammatesinfo_sim) == -1)
    {
        DB_free();
        ROS_ERROR("RTDB发送信息失败，请重启");
        return ;
    }
    else
    {
        ROS_FATAL("message has been PUT to coach");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coachinfo_pub_node");
    ros::NodeHandle n;
    ros::Publisher   coachinfo_pub = n.advertise<nubot_common::CoachInfo>("/receive_from_coach", 1);
    std::string robot_name = argv[1];
    ros::Subscriber  worldmodelinfo_sub = n.subscribe("/" + robot_name + "/" + "worldmodel/worldmodelinfo",1, &receive_worldmodel_info_and_DB_put);
    ros::Rate loop_rate(100);

    /** RTDB通信模块的初始化，开辟内存空间*/
    if(DB_init() != 0)
    {
        ROS_WARN("RTDB没有成功初始化内存空间");
        return 0;
    }
    
    while(ros::ok())
    {
        DB_get_coach_info();
        coachinfo_pub.publish(coach_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


#endif
