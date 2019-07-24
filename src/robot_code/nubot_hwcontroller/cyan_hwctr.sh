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
    rosrun nubot_hwcontroller    nubot_hwcontroller_node ${cyan_prefix}${i}   __name:=${cyan_prefix}_nubot_hwcontroller${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
   sleep 0.5
done 

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait

rosnode cleanup



