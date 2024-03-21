#!/bin/bash

#
# set environment for sick_line_guidance_demo simulation
#

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
export TURTLEBOT3_MODEL=waffle
export SVGA_VGPU10=0           # otherwise "roslaunch turtlebot3_gazebo" may fail when running in VMware
export LIBGL_ALWAYS_SOFTWARE=1 # otherwise "roslaunch turtlebot3_gazebo" may fail when running in VMware

