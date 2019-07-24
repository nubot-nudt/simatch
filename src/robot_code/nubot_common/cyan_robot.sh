#!/bin/bash			
### source the workspace
source ../devel/setup.bash
source devel/setup.bash

### Get parameters and init
declare -i j
declare -i kill_num
cyan_prefix=$(rosparam get /cyan/prefix)
cyan_num=$(rosparam get /cyan/num)
kill_num=0                                  

### spawn cyan robots
for ((i=1; i<=cyan_num; ++i))
do
    rosrun nubot_control    nubot_control_node ${cyan_prefix}${i}   __name:=${cyan_prefix}_nubot_control${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
   
    rosrun world_model      world_model_node   ${cyan_prefix}${i}    __name:=${cyan_prefix}_world_model${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
   
    rosrun nubot_hwcontroller    nubot_hwcontroller_node ${cyan_prefix}${i}   __name:=${cyan_prefix}_nubot_hwcontroller${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"

   sleep 0.5
done 

######### Don't to use RTDB for convenience. Use "rostopic pub" to publish game control
########  info instead.

### run rtdt comm
#rosrun world_model comm &
#PIDS[kill_num]=$!
#let "kill_num=kill_num+1"

### run coachinfo_publisher
# j=2
# rosrun  simulation_interface coach_robot_comm_RTDB ${cyan_prefix}${j} __name:=${cyan_prefix}_coach_robot_comm_RTDB &
# PIDS[kill_num]=$!
# let "kill_num=kill_num+1"

### run strategy_pub_node
rosrun simulation_interface strategy_pub_node ${cyan_prefix} __name:=${cyan_prefix}_strategy_pub &
PIDS[kill_num]=$!
let "kill_num=kill_num+1"

### Optional: run joystick drivers to control football movement
# rosrun joy joy_node  &
# PIDS[kill_num]=$!

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait

rosnode cleanup

### kill nodes
#for ((i=1; i<=cyan_num; ++i))
#do
#	j=$i+1 
#	rosnode kill world_model${j}
#	rosnode kill nubot_control${j}
#done



