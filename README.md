--------------------------
# GAZEBO
## Note: This is for SIMULATION. The following packages should be used together:
1. gazebo_visual                    # for gazebo visulization
2. nubot_ws			                # for real robot code
3. coach_ws_no_rtdb		            # for sending game command such as kickoff to robots; this does not require rtdb

## Configuration of computer A and computer B

>   The recommended way to run simulation is with two computers running nubot_ws and gazebo_visual seperately.

> For example,computer A runs gazebo_visual to display the movement of robots. Computer B runs nubot_ws to calculate and send  movement commands to robots. In addition, computer B should also run coach to send game command such as game start. 

>   The communication between computer A and computer B is via ROS master. The following is the configuration steps:
    
1. In computer A, add computer B's IP address in /etc/hosts; and in computer B, add computer A's IP address in /etc/hosts
e.g. In computer A, `$ sudo gedit /etc/hosts and add "Maggie 192.168.8.100"`
     In computer B, `$ sudo gedit /etc/hosts and add "Bart   192.168.8.101"`
2. In computer A, run gazebo_visual; In computer B, before you run nubot_ws, you should export ROS_MASTER_URI.
e.g. In computer B, ` $ export ROS_MASTER_URI=http://Bart:11311`
3. In computer B, run coach and send game command

### Compiling nubot_ws
#### Method 1:
1. $ sudo chmod +x configure
2. $ ./configure
3. $ catkin_make --pkg nubot_common
4. $ catkin_make --pkg world_model
5. $ catkin_make

#### Method 2:
$ catkin_make -j1

### To run nubot_ws
1. $ export AGENT=1
2. if rtdb is not used, go to step 3; otherwise, run this command: 
   $ rosrun world_model comm
3. $ ./cyan_spawn_model.sh   or   $ ./magenta_spawn_model.sh

### To run gazeo_visual
1. ./configure
2. $ roslaunch nubot_gazebo game_ready.launch
You can edit global_config to change simulation parameters such as the number of robots

### To run coach_ws_no_rtdb
1. $ roslaunch nubot_coach nubot_coach.launch


> **NOTE:** if used with RTDB, then you should change absolute path in rtdb files:

> 1. rtdb/parser/CMakeLists.txt:

>    (1) SET(RTDB_USER_H_FILE   /home/nubot8/nubot_ws/src/nubot/world_model/include/nubot/rtdb/rtdb_user.h)

>    (2) SET(RTDB_INI_FILE      /home/nubot8/nubot_ws/src/nubot/world_model/config/rtdb.ini )

>    (3) SET(RTDB_Config_FILE   /home/nubot8/nubot_ws/src/nubot/world_model/config/rtdb.config )

> 2. rtdb/rtdb_api.cpp:    
>    std::string ini_config_file="/home/nubot8/nubot_ws/src/nubot/world_model/config/rtdb.ini";
> 3. rtdb/comm.cpp: 'wlano' or 'wlan1'
> 4. coach_info_pub.cpp: DB_get(AGENT), this AGENT depends your coach's agent number

--------------------------
## MSL soccer robot code of NuBot team of National University of Defense Technology
# Note
	The strategy part in nubot_control package has been removed. 
	But you can still compile the package successfully.

# Compile
   `$ sudo chmod +x configure`
   
   `$ ./configure`
   
   `$ catkin_make -j1`

# RUN
1. All components:
   $ roslaunch nubot_common nubot.launch
2. Joy stick and hardware controller
   $ roslaunch nubot_hwcontroller nubot_hwcontroller.launch
   $ rosrun nubot_hwcontroller nubot_teleop_joy
3. Show images from the cameras
   $ rqt_image_view

# Error & Fix
1. nubot_hwcontroller shows an error: 'pid *** died'.
Fix: $ sc devel/lib/nubot_hwcontroller/nubot_hwcontroller_node
2. help files are located in /doc folder; please refer to them

# Note
1. If you want to use the simulation function, please edit the file: src/nubot/nubot_common/core/include/nubot/core/core.hpp, and uncomment '#define SIMULATION'. Then compile the code again.For furthur tutorial, see related documentation in other repositories(i.e. 'gazebo_visual' or 'single_nubot_gazebo')
2. contact info: nubot.nudt@outlook.com

--------------------------
# Coach
# Note:
This coach(coach4sim) is modified from coach_ws which is the version for real robots competition. Coach4sim uses ROS topic/messages instead of RTDB.    

# COMPILE
1. `$ catkin_make --pkg nubot_common`   
2. `$ catkin_make`   

# RUN
1. `$ source devel/setup.bash`   
2. run coach for simulation   
	(1) coach for cyan: 	`$ roslaunch nubot_coach cyan_sim.launch`   
	(2) coach for magenta: 	`$ roslaunch nubot_coach magenta_sim.launch`    

# Build error fix:
1. When catkin_make, if it shows "fatal error: Eigen/Eigen: No such file or directory"   
Solution 1: Change all 'Eigen3' to 'Eigen' in CMakeLists.txt of world_model package   
Solution 2: look at /usr/include/eigen3/Eigen, if this folder exists, it means you have already installed Eigen;    
Input this command: $ sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen   

2. if you only changed the configuration parts of some files and git should not record these changes,    
then you could run 'configure' file like this:   
`$ sudo chmod +x configure`   
`$ ./configure`   

# 中国机器人大赛中型组仿真比赛
## 说明
该软件coach4sim一般情况下在比赛中不会用到，它是用来给机器人程序发送比赛开始、暂停、站位等指令，以及显示机器人的状态等等，主要用于对自己的代码进行调试，查看效果而已。关于其   
本软件由RoboCup队伍NuBot的coach直接改过来用于仿真的。欢迎参赛选手共同开发。
## 共同开发
本软件使用git作为版本控制，托管于GitHub网站。如果想共同开发该软件包，请fork一下该软件包（首先你要注册一个GitHub帐号），然后git clone到自己的电脑去，改进完代码后，   
可以git push上来，然后再在GitHub里面发一个pull request。   
## 联系
软件维护者(Maitainer): abcgarden@126.com
Nubot队伍(RoboCup team): nubot.nudt@outlook.com
