#!/bin/bash

source ./simu_env.bash
source ../../../../../install/setup.bash

# rosrun rviz rviz &
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
# rqt_plot /gazebo/model_states/pose[1]/position/x /gazebo/model_states/pose[1]/position/y & # display turtlebots (x,y) position over time

rostopic list
rostopic echo /cmd_vel &
rostopic echo /fsm &
rostopic echo /iam_state &
# rostopic echo /odom &
# rostopic echo /ols &
# rostopic echo /gazebo/model_states
# rostopic echo /gazebo/link_states

