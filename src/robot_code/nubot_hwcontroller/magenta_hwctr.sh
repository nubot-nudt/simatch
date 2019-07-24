#!/bin/bash			

### source the workspace
source ../devel/setup.bash
source devel/setup.bash

### Get parameters and init
declare -i j
declare -i kill_num
magenta_prefix=$(rosparam get /magenta/prefix)
magenta_num=$(rosparam get /magenta/num)
kill_num=0                                  

### spawn magenta robots
for ((i=1; i<=magenta_num; ++i))
do
    rosrun nubot_hwcontroller    nubot_hwcontroller_node ${magenta_prefix}${i}   __name:=${magenta_prefix}_nubot_hwcontroller${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    sleep 0.5
done 

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait

rosnode cleanup
