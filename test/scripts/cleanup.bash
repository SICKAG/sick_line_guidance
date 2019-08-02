#!/bin/bash
pushd ../../../../
echo -e "\n# cleanup.bash: Stopping rosmaster and all rosnodes...\n# rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5"
rosnode kill -a ; sleep 5 ; killall roslaunch ; sleep 5 ; killall rosmaster ; sleep 5
echo -e "\n# cleanup.bash: Deleting ros cache and logfiles and catkin folders ./build ./devel ./install"
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
# rm -rf ~/.ros/log/*
catkin clean --yes --all-profiles --verbose
catkin_make clean
popd

