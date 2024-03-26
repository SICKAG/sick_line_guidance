#!/bin/bash
source ../../../../install/setup.bash
printf "\033c"

# display PointCloud2 messages
rosrun rviz rviz &

# plot sensor positions
rqt_plot /ols/position[0] /ols/position[1] /ols/position[2] &
# rqt_plot /cloud &

# print some values from the object directory of the can device
rostopic list
rostopic echo -n 1 /node1_1001 & rostopic echo -n 1 /node1_1018sub1 & rostopic echo -n 1 /node1_1018sub4
rosservice call /driver/get_object node1 1001sub  false
rosservice call /driver/get_object node1 1018sub1 false
rosservice call /driver/get_object node1 1018sub4 false

# print ros topics for ols and mls
rostopic echo /mls & rostopic echo /ols & rostopic echo /cloud & rostopic echo /diagnostics

