#!/bin/bash
echo -e "makeall.bash: build and install ros sick_line_guidance driver"
sudo echo -e "makeall.bash started."
source /opt/ros/noetic/setup.bash
./cleanup.bash
./makepcan.bash
./make.bash
echo -e "makeall.bash finished."

