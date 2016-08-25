/*
 * Copyright (C) 2015 NuBot team of National University of Defense Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: nubot teleoperation by keyboard.
 * Author: Weijia Yao
 * Date: Jun 2015
 */

// NOTICE: EXCEPT KICK_BALL_VEL,
// others use 'cm' as length unit

#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <iostream>

#include "nubot_teleop_keyboard.hh"

#define RUN -1
#define FLY 1

#define KEYCODE_R  0x43  // right arrow 
#define KEYCODE_L  0x44  // left arrow
#define KEYCODE_U  0x41  // up arrow
#define KEYCODE_D  0x42  // down arrow
#define KEYCODE_Z  0x7a  // key "z" dribble ball switch
#define KEYCODE_X  0x78  // key "x" kick_mode switch
#define KEYCODE_RO 0x2c  // key "," rotate
#define KEYCODE_RR 0x2e  // key "." rotate
#define KEYCODE_SP 0x20    // key " " kick ball

#define VELOCITY        100         // cm/s
#define OMEGA           1.0         // rad/s
#define KICK_BALL_VEL   10.0        // NOTE: m/s

using namespace std;

int kfd = 0;
struct termios cooked, raw;
std::string robot_num;

using namespace nubot;

NubotTeleopKey::NubotTeleopKey():
  vx_(0.0), vy_(0.0), w_(0.0), mode_(RUN),
  dribble_flag_(false), shot_flag_(false)
{
    nh_.param("/cyan_prefix",  robot_prefix_, std::string("nubot"));
    std::string robot_name = robot_prefix_ + robot_num;
    // set param to loose the control requirements
    // nh_.setParam("/nubot/dribble_thres", 0.60);
    // nh_.setParam("/nubot/angle_err_deg", 15);

    vel_pub = nh_.advertise<nubot_common::VelCmd>(robot_name + "/nubotcontrol/velcmd", 10);
    ballhandle_client_ =  nh_.serviceClient<nubot_common::BallHandle>(robot_name + "/BallHandle");
    shoot_client_ = nh_.serviceClient<nubot_common::Shoot>(robot_name + "/Shoot");
}

void NubotTeleopKey::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move nubot.");
  puts("z     -- dribble ball switch.");
  puts("x     -- kick ball mode switch.");
  puts(",     -- rotate nubot counter-clockwise.");
  puts(".     -- rotate nubot clockwise.");
  puts("space -- kick ball.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    vx_ = vy_ = w_ = 0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    last_dribble_flag_ = dribble_flag_;
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        vx_ = -VELOCITY;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        vx_ = VELOCITY;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        vy_ = VELOCITY;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        vy_ = -VELOCITY;
        dirty = true;
        break;
      case KEYCODE_RO:
        w_  = OMEGA;
        dirty = true;
        break;
      case KEYCODE_RR:
        w_ = -OMEGA;
        dirty = true;
        break;
      case KEYCODE_Z:
         dribble_flag_ = !dribble_flag_;
         dirty = true;
         break;
      case KEYCODE_X:
        if(mode_ == RUN)
        {
            mode_ = FLY;
            ROS_INFO("kick ball mode: FLY");
        }
        else
        {
            mode_ = RUN;
            ROS_INFO("kick ball mode: RUN");
        }
        dirty = true;
        break;
      case KEYCODE_SP:
        shot_flag_ = true;
        ROS_INFO("shoot ball!");
        dirty = true;
        break;
      default:
        dirty = false;
        break;
    }
   
    if(dirty ==true)
    {
        // publish movement messages
        vel_cmd_.Vx = vx_;
        vel_cmd_.Vy = vy_;
        vel_cmd_.w = w_;
        vel_pub.publish(vel_cmd_);

        // rosservice call
//        ROS_INFO("last_dribble: %d dribble:%d", last_dribble_flag_, dribble_flag_);
        if(last_dribble_flag_ != dribble_flag_)
        {
            nubot_common::BallHandle b;
            b.request.enable = dribble_flag_;
            ballhandle_client_.call(b);
            if(dribble_flag_)
            {
                if(b.response.BallIsHolding)
                    ROS_INFO("dribble_flag_ : 1");
                else
                {
                    dribble_flag_ = 0;
                    last_dribble_flag_  = dribble_flag_;
                    ROS_INFO("dribble fails. dribble_flag_ : 0");
                }
            }
            else
                ROS_INFO("dribble_flag_ : 0");
        }

        if(shot_flag_)
        {
            nubot_common::Shoot s;
            s.request.ShootPos = mode_;
            s.request.strength = KICK_BALL_VEL;
            shoot_client_.call(s);
            shot_flag_ = false;
            if(s.response.ShootIsDone)
                ROS_INFO("shoot ball success!");
            else
                ROS_INFO("shoot ball failure");
        }

        dirty=false;
    }
  }
  return;
}

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  cout<<"Input robot number"<<endl;
  cin>>robot_num;

  ros::init(argc, argv, "teleop_nubot", ros::InitOption::AnonymousName);
  NubotTeleopKey teleop_nubot;

  signal(SIGINT,quit);
  teleop_nubot.keyLoop();

  return(0);
}




