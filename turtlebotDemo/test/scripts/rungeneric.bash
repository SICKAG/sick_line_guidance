#!/bin/bash

#
# set environment, clear screen and logfiles
#

if [ ! -d ./log ] ; then mkdir -p ./log ; fi 
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi 
printf "\033c"
rm -rf ./log/*
source ./simu_env.bash
pushd ../../../../..
rm -rf ~/.ros/log/*
# printf "\033c"
source /opt/ros/melodic/setup.bash
source ./install/setup.bash
./src/sick_line_guidance/turtlebotDemo/test/scripts/killall.bash
export LIBPROFILER=/usr/lib/x86_64-linux-gnu/libprofiler.so.0 # path to libprofiler.so.0 may depend on os version

#
# Start fsm, driver, nodes, tools and run the sick_line_guidance_demo simulation
#

# roscore &
# sleep 10 # wait for roscore startup processes

if [ "$TURTLEBOT_SIMULATION" == "1" ] ; then

  # Start gazebo / turtlebot simulator
  sleep 1 ; echo -e "#\n# runsimu.bash: starting gazebo ..."
  sleep 1 ; roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
  sleep 10 # wait until gazebo has started ...
  # Virtual navigation:
  # roslaunch turtlebot3_gazebo turtlebot3_world.launch
  # roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
  export SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS='ols_simu:=1'

else # "$TURTLEBOT_SIMULATION" == "0"

  if [ "$TURTLEBOT_STOP_AT_START" == "1" ] ; then # Stop turtlebot (set all velocities to 0 and restart turtlebot)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch &
    sleep 15 ; rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' # stop the turtlebot
    sleep 1 ; rosnode kill turtlebot3_core ; sleep 1 ; rosnode kill turtlebot3_diagnostics
  fi

  # Init turtlebot
  sleep 5 ; roslaunch turtlebot3_bringup turtlebot3_robot.launch &
  sleep 15

  # Start OLS20 driver or simulated OLS messages
  if [ "$TURTLEBOT_OLS_SIMULATION" == "1" ] ; then # Run TurtleBot with simulated OLS messages
    export SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS='ols_simu:=1'
  else #  "$TURTLEBOT_OLS_SIMULATION" == "0" # CAN initialization and start OLS20 driver 
    # Init CAN and start OLS20 driver
    sudo ip link set can0 up type can bitrate 125000 # configure can0
    ip -details link show can0 # check status can0
    # Start OLS20 driver
    sleep 5 ; roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml > ~/.ros/log/sick_line_guidance_ols20.log & 
    sleep 10 ; export SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS='ols_simu:=0'
  fi # "$TURTLEBOT_OLS_SIMULATION"

fi # "$TURTLEBOT_SIMULATION"

# Start sick_line_guidance_demo simulation
export SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS=$SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS' visualize:='$TURTLEBOT_VISUALIZE
echo -e "#\n# runsimu.bash: starting simulation:\n# roslaunch sick_line_guidance_demo sick_line_guidance_demo_node.launch $SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS \n#"

if [ "$TURTLEBOT_PROFILING" == "1" ] ; then   # debug and profile sick_line_guidance_demo
  rm -f /tmp/sick_line_guidance_demo.prof*    # remove previous profile logs
  env CPUPROFILE=/tmp/sick_line_guidance_demo.prof LD_PRELOAD=$LIBPROFILER roslaunch sick_line_guidance_demo sick_line_guidance_demo_node.launch $SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS &
else  # run sick_line_guidance_demo
  roslaunch -v --screen sick_line_guidance sick_line_guidance_demo_node.launch $SICK_LINE_GUIDANCE_DEMO_LAUNCH_ARGS 2>&1 | tee ~/.ros/log/sick_line_guidance_demo_node.log &
fi
sleep 10

# Start sick_line_guidance watchdog run emergency exit to stop the TurtleBot after line lost for more than 1 second
if ! [ "$TURTLEBOT_START_WATCHDOG_AFTER" == "" ] && [ $TURTLEBOT_START_WATCHDOG_AFTER -ge 0 ] ; then
  sleep $TURTLEBOT_START_WATCHDOG_AFTER # start watchdog after a simulated OLS is close enough to a line
  roslaunch -v --screen sick_line_guidance sick_line_guidance_watchdog.launch 2>&1 | tee ~/.ros/log/watchdog.log & # run watchdog
fi

#
# Run sick_line_guidance_demo for a while: just wait for key 'q' or 'Q'
# while ros nodes sick_line_guidance_demo_node and Robot_FSM are running
#

while true ; do  
  echo -e "FSM and sick_line_guidance_demo simulation running. Press 'q' to exit..." ; read -t 0.1 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
done

#
# Stop turtlebot, fsm and sick_line_guidance_demo
#

rosnode kill sick_line_guidance_demo_node
sleep 0.5 ; rostopic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}' ; sleep 0.5 # stop the turtlebot
./src/sick_line_guidance/turtlebotDemo/test/scripts/killall.bash

#
# Check errors and warnings in ros logfiles
#
echo -e "\n#\n# sick_line_guidance_demo simulation finished.\n#\n"
grep "\[ WARN\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*.log >> ~/.ros/log/ros_log_warnings.txt
grep "\[ERROR\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*.log >> ~/.ros/log/ros_log_errors.txt
echo -e "\nSimulation warnings and errors:\n"
cat ~/.ros/log/ros_log_warnings.txt
cat ~/.ros/log/ros_log_errors.txt

#
# Profiling 
#
if [ "$TURTLEBOT_PROFILING" == "1" ] ; then
  for proffile in /tmp/robot_fsm.prof* ; do 
    google-pprof --pdf ./install/lib/iam/robot_fsm $proffile > $proffile.pdf
  done
  for proffile in /tmp/sick_line_guidance_demo.prof* ; do 
    google-pprof --pdf ./install/lib/sick_line_guidance_demo/sick_line_guidance_demo_node $proffile > $proffile.pdf
  done
fi

#
# Play video recorded by sick_line_guidance_demo
#
popd
cp ~/.ros/log/watchdog.log                     ./log
cp ~/.ros/log/robot_fsm.log                    ./log
cp ~/.ros/log/sick_line_guidance_demo_node.log ./log
if [ "$TURTLEBOT_PROFILING" == "1" ] ; then
  cp /tmp/robot_fsm.prof*.pdf                  ./log
  cp /tmp/sick_line_guidance_demo.prof*.pdf    ./log
fi
if [ -f ~/.ros/sick_line_guidance_demo.avi ] ; then 
  mv ~/.ros/sick_line_guidance_demo.avi ./log/sick_line_guidance_demo.avi
  echo -e "\nCreated sick_line_guidance_demo.avi:"
  ls -al ./log/sick_line_guidance_demo.avi
  echo -e "ffplay ./sick_line_guidance_demo.avi ; # Use right mouse button to seek forward/backward, 'p' for pause/resume"
  ffplay ./log/sick_line_guidance_demo.avi
  echo -e "\n"
fi
ls -al ./log/*

