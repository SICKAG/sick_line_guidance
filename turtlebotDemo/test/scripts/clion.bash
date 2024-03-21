#!/bin/bash

# init ros environment
source /opt/ros/melodic/setup.bash
source ../../../../../devel/setup.bash
source ../../../../../install/setup.bash

# start edit resource-files
gedit ./rungeneric.bash ../../sick_line_guidance_demo/launch/sick_line_guidance_demo_node.launch ../../sick_line_guidance_demo/yaml/sick_line_guidance_demo.yaml &

# start clion
echo -e "Starting clion...\nNote in case of clion/cmake errors:"
echo -e "  Click 'File' -> 'Reload Cmake Project'"
echo -e "  cmake/clion: Project 'XXX' tried to find library '-lpthread' -> delete 'thread' from find_package(Boost ... COMPONENTS ...) in CMakeLists.txt"
echo -e "  rm -rf ../../../.idea # removes all clion settings"
echo -e "  rm -f ~/CMakeCache.txt"
echo -e "  'File' -> 'Settings' -> 'CMake' -> 'CMake options' : CATKIN_DEVEL_PREFIX=~/TASK007_PA0100_ROS_Treiber_Spurfuehrungsssensor/catkin_ws/devel"
echo -e "  'File' -> 'Settings' -> 'CMake' -> 'Generation path' : ../clion_build"

pushd ../../../../..
~/Public/clion-2018.3.3/bin/clion.sh ./src &
popd

