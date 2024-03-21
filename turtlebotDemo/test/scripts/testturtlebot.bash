#!/bin/bash

#
# simple turtlebot test, drives a figure of eight
#

#
# set environment, clear screen and logfiles
#

export TURTLEBOT_SIMULATION=0 # 1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
export TURTLEBOT3_MODEL=waffle
export SVGA_VGPU10=0           # otherwise "roslaunch turtlebot3_gazebo" may fail when running in VMware
export LIBGL_ALWAYS_SOFTWARE=1 # otherwise "roslaunch turtlebot3_gazebo" may fail when running in VMware

if [ ! -d ./log ] ; then mkdir -p ./log ; fi 
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi 
printf "\033c"
rm -rf ./log/*
pushd ../../../../..
rm -rf ~/.ros/log/*
source /opt/ros/melodic/setup.bash
source ./install/setup.bash
rosnode kill -a
sleep 5

#
# Start turtlebot resp. turtlebot simulation
#

if [ "$TURTLEBOT_SIMULATION" == "1" ] ; then
  rosnode kill -a ; sleep 1 ; killall rqt_plot gzclient ; sleep 1 ; killall gzserver ; sleep 3
  roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 2>&1 &
else
  roslaunch turtlebot3_bringup turtlebot3_robot.launch 2>&1 | tee ~/.ros/log/turtlebot3_bringup.log &
  # Init CAN and start OLS20 driver
  sudo ip link set can0 up type can bitrate 125000 # configure can0
  ip -details link show can0 # check status can0
  sleep 1 ; roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml > ~/.ros/log/sick_line_guidance_ols20.log & 
fi
sleep 15

#
# Send some cmd_vel messages to test
#

# for ((i=1;i<=20;i++)) ; do  
#   rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: -0.4}}' &
#   sleep 0.1
# done
# for ((i=1;i<=20;i++)) ; do  
#   rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' &
#   sleep 0.1
# done


#
# Start turtlebot_test_fsm_node
#

roslaunch -v --screen sick_line_guidance_demo turtlebot_test_fsm_node.launch & # 2>&1 | tee ~/.ros/log/turtlebot_test_fsm_node.log &
# roslaunch -v --screen sick_line_guidance_demo sick_line_guidance_demo_node.launch ols_simu:=0 visualize:=0 2>&1 | tee ~/.ros/log/sick_line_guidance_demo_node.log &

#
# Run turtlebot_test_fsm_node for a while: just wait for key 'q' or 'Q'
# while ros node turtlebot_test_fsm_node is running
#

while true ; do  
  echo -e "turtlebot_test_fsm_node running. Press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
done

#
# Stop turtlebot_test_fsm_node
#

rosnode kill turtlebot_test_fsm_node
sleep 0.5 ; rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' ; sleep 0.5 # stop the turtlebot
rosnode kill -a

popd
cp ~/.ros/log/turtlebot_test_fsm_node.log  ./log
cp ~/.ros/log/sick_line_guidance_ols20.log ./log
cp ~/.ros/log/turtlebot3_bringup.log       ./log
ls -al ./log/*


