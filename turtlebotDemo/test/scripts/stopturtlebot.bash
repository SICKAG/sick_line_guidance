#!/bin/bash

#
# stop the turtlebot
#

# environment
pushd ../../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash
echo -e "#\n#stop turtlebot...\n#"

# kill all ros nodes
rosnode kill -a
sleep 0.3
killall -9 robot_fsm

# init turtlebot and send velocity 0.0
roslaunch turtlebot3_bringup turtlebot3_robot.launch &
sleep 1
for ((i=1;i<=20;i++)) ; do  
  rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}'
  sleep 0.5
done

# stop turtlebot ros node
rosnode kill -a

