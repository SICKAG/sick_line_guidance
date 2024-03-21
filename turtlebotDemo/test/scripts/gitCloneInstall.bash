#!/bin/bash

#
# Get all packages for sick_line_guidance_demo,
# build and install.
#

# Clean up
# rosclean purge -y
# rm -rf ./build ./devel ./install
# rm -rf ~/.ros/*
# catkin clean --yes --all-profiles --verbose
# catkin_make clean 
#
# Install ros packages for turtlebot
pushd ../../../../../src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone https://github.com/ros-drivers/rosserial.git
# Install ros packages for turtlebot simulation
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin.git
# Install can_open packages
git clone https://github.com/ros-industrial/ros_canopen.git
git clone https://github.com/CANopenNode/CANopenSocket.git
git clone https://github.com/linux-can/can-utils.git
# Install sick_line_guidance package
git clone https://github.com/ros-planning/random_numbers.git
git clone https://github.com/SICKAG/sick_line_guidance.git
# Install ros packages required for robot_fsm
git clone https://github.com/uos/sick_tim.git
# Install video support for sick_line_guidance_demo
sudo apt-get install ffmpeg
sudo apt-get install vlc
# Install profiling and performance tools
git clone https://github.com/catkin/catkin_simple.git
# git clone https://github.com/ethz-asl/schweizer_messer.git # toolbox including timing utilities
sudo svn export https://github.com/ethz-asl/schweizer_messer/trunk/sm_common # common utilities from ethz-asl "schweizer messer" toolbox
sudo svn export https://github.com/ethz-asl/schweizer_messer/trunk/sm_timing # timing utilities from ethz-asl "schweizer messer" toolbox
sudo apt-get install google-perftools libgoogle-perftools-dev graphviz # libprofiler for profiling
# Build and install
cd ..
catkin_make install --cmake-args -DTURTLEBOT_DEMO="ON"
source ./install/setup.bash
popd
