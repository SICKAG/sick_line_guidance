#!/bin/bash

pushd ../../../../../
source /opt/ros/melodic/setup.bash
./src/sick_line_guidance_demo/test/scripts/killall.bash
killall rosmaster ; sleep 1

echo -e "\n# cleanup.bash: Deleting ros cache and logfiles and catkin folders ./build ./devel ./install"
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
# rm -rf ~/.ros/log/*
catkin clean --yes --all-profiles --verbose
catkin_make clean
popd
if [ -f ./sick_line_guidance_demo.avi ] ; then rm -f ./sick_line_guidance_demo.avi ; fi

