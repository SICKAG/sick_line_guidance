#!/bin/bash

echo -e "#\n# runsimu.bash: Stopping all rosnodes...\n# rosnode kill -a ; sleep 1 ; killall robot_fsm rqt_plot gzclient gzserver ; sleep 3\n#"
rosnode kill -a ; sleep 1 ; killall robot_fsm ; killall rqt_plot gzclient ; sleep 1 ; killall gzserver ; sleep 3  # ; killall rosmaster ; sleep 1

