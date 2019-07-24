#!/bin/bash

### Set the number of robots
declare -i cyan_num=$(rosparam get /cyan/num)
declare -i magenta_num=$(rosparam get /magenta/num)
declare -i j
declare -i kill_num
football_name=$(rosparam get /football/name)
magenta_prefix=$(rosparam get /magenta/prefix)
cyan_prefix=$(rosparam get /cyan/prefix)
		                               
cyan_x=(0 -10.5 -2 -2 -5 -5 )  # the first one is for goal-keeper
cyan_y=(0 0 1 -1 2 -2 )       # the first one is for goal-keeper 
magenta_x=(0 10.5 2 2 5 5)      # the first one is not useful now
magenta_y=(0 0 -1 1 -2 2 )     # the first one is not useful now-keeper

### spawn the football
rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/football/model.sdf -sdf \
                              -model ${football_name} \
                              -x 0.0 -y 0.0 -z 0.0 &    
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"        
    sleep 1

### spawn cyan robots
for ((i=1; i<=cyan_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/nubot${i}/model.sdf -sdf \
                                  -model ${cyan_prefix}${i} \
                                  -x ${cyan_x[$i]} -y ${cyan_y[$i]} -z 0.0 &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    sleep 0.5
done 

### spawn magenta robots
for ((i=1; i<=magenta_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/rival${i}/model.sdf -sdf \
                                  -model ${magenta_prefix}${i} \
                                  -x ${magenta_x[$i]} -y ${magenta_y[$i]} -z 0.0 &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    sleep 0.5
done 

trap 'kill ${PIDS[*]}' SIGINT
wait
rosnode cleanup

### use joystick
# rosrun joy joy_node &

