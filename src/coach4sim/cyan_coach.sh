#!/bin/bash			

### source the workspace
#source ../devel/setup.bash
#source devel/setup.bash

### Get parameters and init
#declare -i kill_num
kill_num=0
cyan_prefix=$(rosparam get /cyan/prefix)

### run
#rosrun   world_model     world_model_node & __name:=$(cyan_prefix)_wm &
#let "kill_num=kill_num+1"

rosrun  nubot_coach nubot_coach_node ${cyan_prefix} __name:=nubot_coach_cyan &
#PIDS[kill_num]=$!
#let "kill_num=kill_num+1"

### kill thoes background processes
#trap 'kill ${PIDS[*]}' SIGINT
#wait

