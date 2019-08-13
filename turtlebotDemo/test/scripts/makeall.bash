#!/bin/bash
echo -e "makeall.bash: build and install ros sick_line_guidance driver and demo"
sudo echo -e "makeall.bash started."
# sudo apt-get install python-catkin-lint
source /opt/ros/melodic/setup.bash
./cleanup.bash
./makepcan.bash
./make.bash
echo -e "makeall.bash finished."

