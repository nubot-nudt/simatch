# 中国机器人大赛中型组仿真比赛
![simatch][pic1]
## 说明
该软件包包括了robot_code模块，gazebo_visual模块，coach4sim模块和common模块，其中，参赛选手主要注重robot_code模块，里面包含的是控制机器人运动的相关程序。各个模块的介绍如下：   

 - robot_code: 机器人的感知、规划、运动控制等方面，由NuBot队伍的机器人代码改造而成,其中主要的策略等(在nubot_control软件包中)部分已经删除，选手应自行编写，其余部分可以直接沿用，也可随意改动。
 -  gazebo_visual: Gazebo仿真平台，除基本的设置（在sim_config文件中）外，不应做其他改动。最终使用版本以比赛时的版本为准。   
 - coach4sim: 用于仿真的coach，发送比赛开始、暂停、站位等指令。
 - common: 包含cmake、手柄驱动等。选手不必做改动。   

> 请先学习ROS、C++，作为基本技能和知识，然后对机器人策略等方面有一定了解，再进行robot_code的多机器人协同程序的编写。
   
## 可能增加或改变的规则(最终以比赛时为准)
1. 比赛不分上下半场，一共15分钟左右；
2. 任何利用自动裁判盒的漏洞获取比赛中的优势将视为作弊行为，比赛成绩无效;   

## 共同开发
欢迎大家积极共同完善该仿真平台，我们将感谢每一个人的贡献。   
本软件使用git作为版本控制，托管于GitHub网站。如果想共同开发该软件包，请fork一下该软件包（首先你要注册一个GitHub帐号），然后git clone到自己的电脑去，改进完代码后，可以git push上来，然后再在GitHub里面发一个pull request。   

> **需要的基本技能和知识:**  
> 
>  - 软件编程： 
> 	 - robot_code: ROS, C++    
> 	 - gazebo_visual: ROS, C++, Gazebo插件以及编程实现    
> 	 - coach4sim:	 ROS, C++, Qt    
>	 - 配置文件:	 Linux bash或其他     
>  - 其他方面   
> 	 - 使用说明:	 txt，Markdown 
> 	 - 软件文档：	 Doxygen或其他

为了方便大家交流，请大家把遇到的问题发到这里<https://github.com/nubot-nudt/simatch/issues>，我会抽时间回答，别人能回答的话也帮忙回答。建议所有人都注册一个github帐号，对这个simatch版本库fork一下。fork的目的在于你对它的修改可以提交给我，然后我审批过后就可以融合进来。目前希望大家能够将自己在使用过程终于遇到的问题以及解决办法写到README.md文件，然后提交给我。如果有参考价值，那么我就把它融合进来，这样所有人都能看到，也是对这个软件的一个贡献。    

**步骤**   
1. 按下图中的fork就行了（前提是你已经登陆了github帐号），这样在你的帐号里就有了simatch。   
![fork][pic4]   
2. 然后点进README.md文件，就可以编辑了。把自己遇到的问题和解决部分写在这里questions & answers如下图：   
![q&a][pic5]   
3. 最后创建一个pull request如下图所示，这样我就能够审批通过你的编辑了   
![pull][pic6]   
   
## 联系
软件维护者(Maitainer): abcgarden@126.com   
Nubot队伍(RoboCup team): nubot.nudt@outlook.com   

-------------------------------------------------
-------------------------------------------------
# 用户手册 User manual
> **NOTE:** 
> If you want to have a basic understanding of how Gazebo and ROS combines to work for robots, it is recommended to check out the repository ['single_nubot_gazebo'][1].
> This contains how to configure the environment, how to run the simulation, and how the robot is simulated. You could run the ROS tool ['rqt_graph'][2] to
> understand the basic messages and service flow. 

## Package Summary   
   
- Maintainer status: maintained
- Maintainer: Weijia Yao <abcgarden@126.com>
- Author: [NuBot Team](https://www.trustie.net/organizations/23?org_subfield_id=108)
- License: Apache
- Bug / feature tracker: https://github.com/nubot-nudt/simatch/issues   
- Source: git https://github.com/nubot-nudt/simatch (branch: master)   

## Recommended Operating Environment
1. Ubuntu 14.04; 
2. ROS Indigo or ROS Jade. (It is recommended to install ROS Jade)
3. Gazebo 5.0 or above;
4. gazebo_ros_pkgs; (please read the **NOTE** below for more information)  
5. If you decide to use coach4sim with a GUI, you should make sure you have installed Qt5. The recommended install place is /opt. 
Other versions of Ubuntu, ROS or Gazebo may also work, but we have not tested yet.

**NOTE:** 
Concerning how to install appropriate **gazebo_ros_pkgs**, please read the following according to your own situation:   
 - 1.  If you decide to use **ROS Indigo**, please read the following:   
If you choose "desktop-full" install of ROS Indigo, there is a Gazebo 2.0 included initially. In order to install Gazebo 5.0/5.1, you should first remove Gazebo 2.0 by running:   
(**The following command is dangerous; it might delete the whole ROS, so please do it carefully or you may find other ways to delete gazebo2**)   
` $ sudo apt-get remove gazebo2* `    
Then you should be able to install Gazebo 5.0 now. To install gazebo_ros_pkgs compatible with Gazebo
5.0/5.1, run this command:   
` $ sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control`   
HOWEVER,    if the above command does now work, these packages may be moved to other places. You can check out [gazebo_ros][3] and download and install the correct version.   
 - 2. If you decide to use **ROS Jade** with **gazebo 5.0 or 5.1**, read the following   
ROS Jade has gazebo_ros_pkgs with it; so you don't have to install gazebo_ros_pkgs again.  
However, you should do the following steps to fix some of the bugs in ROS Jade related to Gazebo:        
  -  (a) `$ sudo gedit /opt/ros/jade/lib/gazebo_ros/gazebo`    
In this file, go to line 24 and delete the last '/'. So    
`setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo/`    
is changed to     
`setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo`    
You can read this link for more [information][4]
  -  (b) Install Gazebo 5.     
   `$ sudo apt-get install gazebo5`     
If this fails, try to run the ['gazebo5_install.sh'][5](obtained from Gazebo's official website).    
Read for more [information][6]   
  -  (c) Optional: copy resource files to the new gazebo folder.    
   `$ sudo cp -r /usr/share/gazebo-5.0/* /usr/share/gazebo-5.1`      
 - 3. If you decide to use **ROS Jade** with **gazebo 7.1**, read the following,    
  -  (1) Install gazebo 7.0 by running [gazebo7_install.sh][7](obtained from Gazebo's official website);      
  -  (2) Then run this in the terminal:   
  -  ` $ sudo apt-get install ros-jade-gazebo7-ros-pkgs`   

## Compile
1. ` $ sudo chmod +x configure`
2. ` $ ./configure`
3. ` $ catkin_make `  

**Compile Error Solution:**   
1) Cannot find cmake file related to Qt and therefore cannot complie coach4sim.   
  + Explanation: This means it cannot find the cmake file to Qt. Since coach4sim uses Qt to draw its GUI, we need Qt to compile the program successfully(solution 1). However, since the function of coach4sim is only to send game commands, you could do this manually by sending approriate ROS messages without using coach4sim(solution 2).   
  + Solution 1:  You should first install Qt and then add the location to CMAKE_PREFIX_PATH. In this case, go to src/coach4sim/CMakeLists.txt and add the path to line 5. The final result would look like this:   
 `  set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} "/opt/Qt5.3.2/5.3/gcc_64/lib/cmake/Qt5Widgets/") `    
  + Solution 2: In another terminal, input the following to send a game command:   
```    
rostopic pub -r 1 /nubot/receive_from_coach  nubot_common/CoachInfo "
MatchMode: 10
MatchType: 0" 
```   
  Indeed, when you input until nubot_common/CoachInfo, you could press 'Tab' twice and then the whole definition of the message would show up. Then you could fill up the message. However, you only need to fill in two fields: 'MatchMode' and 'MatchType', where 'MatchMode' is the current game command, 'MatchType' is the previous game command. The coding of the game commands is in core.hpp. For quick reference:   
```   
enum MatchMode {
                  STOPROBOT  =  0,
                  OUR_KICKOFF = 1,
                  OPP_KICKOFF = 2,
                  OUR_THROWIN = 3,
                  OPP_THROWIN = 4,
                  OUR_PENALTY = 5,
                  OPP_PENALTY = 6,
                  OUR_GOALKICK = 7 ,
                  OPP_GOALKICK = 8,
                  OUR_CORNERKICK = 9,
                  OPP_CORNERKICK = 10,
                  OUR_FREEKICK = 11,
                  OPP_FREEKICK = 12,
                  DROPBALL     = 13,
                  STARTROBOT   = 15,
                  PARKINGROBOT = 25,
                  TEST = 27
                };
```   
2) When catkin_make, if it shows "fatal error: Eigen/Eigen: No such file or directory"   
  + Solution 1: Change all 'Eigen3' to 'Eigen' in CMakeLists.txt of world_model package in robot_code module  
  + Solution 2: look at /usr/include/eigen3/Eigen, if this folder exists, it means you have already installed Eigen;    
Input this command:    
`$ sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen`    

3) if you come across other problems, you could refer to doc/ folder.    

--------------------------
## Run
  You could run the robot_code, gazebo_visal and coach4sim in one computer. However, the simulation speed would be low. So you could also run them with several computers to decrease the burden of one single computer.
### With one computer
Assuming you are in the root directory of 'simatch',   
If you want to run those modules in one launch, you could   
1 `$ source devel/setup.bash` 

> You could write this line into ~/.bashrc file so that you don't have to source it every time you open a terminal. 

2 `$ roslaunch simatch_cyan.launch` or `$ roslaunch simatch_magenta.launch`   


If you want to run those modules seperatly, you could   
1 `$ source devel/setup.bash`   

> You could write this line into ~/.bashrc file so that you don't have to source it every time you open a terminal.   

2 To run gazebo_visual,    
`$ roslaunch nubot_gazebo game_ready.launch`    
3 To run robot_code for cyan or magenta robots:      
`$ rosrun nubot_common cyan_robot.sh` or `$ rosrun nubot_common magenta_robot.sh`   
4 To run coach4sim,      
`rosrun coach4sim cyan_coach.sh` or `rosrun coach4sim magenta_coach.sh`   

> **NOTE:** 
> 1. You could change some parameters in sim_config file and relaunch all modules again.
> 2. You might not watch the robots doing anything because the movement part in 'nubot_control' package is removed. You might need to write some codes by yourself.

### With several computers

**Configuration of computer A and computer B**   
![multi-computers][pic3]
  The recommended way to run simulation is with two computers running nubot_ws and gazebo_visual seperately.   
For example,computer A runs gazebo_visual to display the movement of robots. Computer B runs nubot_ws to    
calculate and send  movement commands to robots. In addition, computer B should also run coach to send game    
command such as game start.    
  The communication between computer A and computer B is via ROS master. 
  The following is the configuration steps:   
> 1. In computer A, add computer B's IP address in /etc/hosts; and in computer B, add computer A's IP address in /etc/hosts >  
> e.g. In computer A, `$ sudo gedit /etc/hosts and add "Maggie 192.168.8.100"`   
>     In computer B, `$ sudo gedit /etc/hosts and add "Bart   192.168.8.101"`   
> 2. In computer A, run gazebo_visual; In computer B, before you run nubot_ws, you should export ROS_MASTER_URI.   
> e.g. In computer B, ` $ export ROS_MASTER_URI=http://Bart:11311`   
> 3. In computer B, run coach and send game command   

## Tutorial
### ROS topics, messages and services
The robot movement is realized by a Gazebo model plugin which is called "NubotGazebo" generated by source files "nubot_gazebo.cc" and "nubot_gazebo.hh". Basically the essential part of the plugin is realizing basic motions: omnidirectional locomotion, ball-dribbling and ball-kicking.    
![rosgraph][pic2]
   
The gazebo plugin subscribes to topic **"nubotcontrol/velcmd"** for omnidirecitonal movement and **"omnivision/OmniVisionInfo"** which contains messages about the soccer ball and all the robots' information such as position, velocity and etc. like the functions of an omnivision camera. For services, it subscribes to service **"BallHandle"** and **"Shoot"** for ball-dribbling and ball-kicking respectively.  Since there may be multiple robots, these topics or services names should be prefixed with the robot model names in order to distinguish between each other. For example, if your robot model's name is "nubot1", then the topic names are **"/nubot1/nubotcontrol/velcmd"** and **"/nubot1/omnivision/OmniVisionInfo"** and the service names would be changed to **"/nubot1/BallHandle"** and **"/nubot1/Shoot"** accordingly. You can customize this code for your robot based on these messages and services as a convenient interface. The types and definitions of the topics and servivces are listed in the following table:    


Topic/Service	|	Type	|	Definition |
:-------------: |:-------:|:------------|
**/nubot1/nubotcontrol/velcmd**	|	nubot_common/VelCmd 	|	float32 Vx <br> float32 Vy <br>  float32 w   |
**/nubot1/omnivision/OmniVisionInfo** | ubot_common/OminiVisionInfo | Header header <br> [BallInfo][8] ballinfo <br> [ObstaclesInfo][9] obstacleinfo <br> [RobotInfo][10][]  robotinfo |
**/nubot1/BallHandle**   |  nubot_common/BallHandle       |  int64 enable <br> --- <br>  int64 BallIsHolding |
**/nubot1/Shoot**        |  nubot_common/Shoot            | int64 strength <br> int64 ShootPos <br>  --- <br> int64 ShootIsDone |   
   
      
For the definition of /BallHandle service, when "enable" equals to a non-zero number, a dribble request would be sent. If the robot meets the conditions to dribble the ball, the service response "BallIsHolding" is true.    
   
For the definition of /Shoot service, when "ShootPos" equals to -1, this is a ground pass. In this case, "strength" is the inital speed you would like the soccer ball to have. When "ShootPos" equals to 1, this is a lob shot. In this case, "strength" is useless since the strength is calculated by the Gazebo plugin automatically and the soccer ball would follow a parabola path to enter the goal area. If the robot successfully kicks the ball out even if it failed to goal, the service response "ShootIsDone" is true.   

For the definition of the "**omnivision/OmniVisionInfo**" topic, there are three new message types: "BallInfo", "ObstaclesInfo" and "RoboInfo". The field "robotinfo" is a vector. Before introducing the format of these new messages, three other message types "Point2d", "PPoint" and "Angle" are used in their definitions:   
```bash
# Point2d.msg, reperesenting a 2-D point.
float32 x				# x component
float32 y				# y component
```
   
```bash
# PPoint.msg, representing a 2-D point in polar coordinates.
float32 angle				# angle against polar axis
float32 radius				# distance from the origin
```

```bash
# Angle.msg, representing the angle
float32 theta				# angle of rotation
```
-----------------------------------   
   
```bash
# BallInfo.msg, representing the information about the ball
Header header                           # a ROS header message defined by ROS package std_msgs
int32     ballinfostate	                # the state of the ball information; 
Point2d   pos                           # ball position in global reference frame; the origin of the frame is the center of
                                        # the soccer field; x-axis pointed horizentally towards the opponet's center of the
                                        # goal area and y-axis vertiacal to the x-axis using the right-hand rule
PPoint    real_pos                      # ball relative position in robot body frame; the origin of the body frame is the
                                        # center of the robot base; x-axis along the kicking-mechanism and y-axis vertical
                                        # to the x-axis using the right-hand rule
Point2d   velocity                      # ball velocity in global reference frame
bool      pos_known                     # ball position is known(1) or not(0)
bool      velocity_known                # ball velocity is known(1) or not(0)
```

```bash
# ObstaclesInfo.msg, representing the information about obstacles
Header header                           # a ROS header message defined by ROS package std_msgs
Point2d[] pos                           # obstacle position in global reference frame
PPoint[] polar_pos                      # obstable position in the polar frame of which the origin is the center of the
                                        # robot and the polar axis is along the kicking mechanism
```
   
```bash
# RobotInfo.msg, representing teammates' information
Header header                           # a ROS header message defined by ROS package std_msgs
int32    AgentID                        # ID of the robot
int32    targetNum1                     # robot ID to be assigned target position 1
int32    targetNum2                     # robot ID to be assigned target position 2
int32    targetNum3                     # robot ID to be assigned target position 3
int32    targetNum4                     # robot ID to be assigned target position 4
int32    staticpassNum                  # in static pass, the passer's ID
int32    staticcatchNum                 # in static pass, the catcher's ID
Point2d  pos                            # robot position in global reference frame
Angle    heading                        # robot heading in global reference frame
float32  vrot                           # the rotational velcoity in global reference frame
Point2d  vtrans                         # the linear velocity in global reference frame
bool     iskick                         # robot kicks the ball(1) or not(0)
bool     isvalid                        # robot is valid(1) or not(0)
bool     isstuck                        # robot is stuck(1) or not(0)
bool     isdribble                      # robot dribbles the ball(1) or not(0)
char     current_role                   # the current role
float32  role_time                      # time that the robot keeps the role unchanged
Point2d  target	                        # target position
```
   
The units of these elements are cm, cm/s, rad and rad/s.    
   
### Code samples
**Working...... :)**
--------------------------
## Questions & Answers
1. 问题：
解决办法：
   
2. 问题：
解决办法：

[1]: https://github.com/nubot-nudt/single_nubot_gazebo
[2]: http://wiki.ros.org/rqt_graph
[3]: https://github.com/ros-simulation/gazebo_ros_pkgs.git
[4]: http://answers.ros.org/question/215796/problem-for-install-gazebo_ros_package/
[5]: gazebo5_install.sh
[6]: http://answers.ros.org/question/217970/ros-jade-and-gazebo-50-migration-problem/
[7]: gazebo7_install.sh
[8]: src/robot_code/nubot_common/msgs/BallInfo.msg
[9]: src/robot_code/nubot_common/msgs/ObstaclesInfo.msg
[10]: src/robot_code/nubot_common/msgs/RobotInfo.msg
[pic1]: pics/simatch.png
[pic2]: pics/rosgraph_single_robot.png
[pic3]: pics/multi-computers.png
[pic4]: pics/fork.jpg
[pic5]: pics/q&a.jpg
[pic6]: pics/pull_request.png
