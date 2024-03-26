#!/bin/bash

# set environment, clear screen and logfiles
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi 
printf "\033c"
source ../../../../install/setup.bash
echo -e "\n# run.bash: Stopping rosmaster and all rosnodes...\n# rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5"
rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5
rm -rf ~/.ros/log/*

# initialize can net device, can0 by peak can adapter
# sudo ip link set can0 up type can bitrate 125000
NUM_BITRATE_125000=$(ip -details link show can0 | grep "bitrate 125000" | wc -l)
if [ "$NUM_BITRATE_125000" = "0" ] ; then
  echo -e "# run sick_line_guidance:\n# sudo ip link set can0 up type can bitrate 125000"
  sudo ip link set can0 up type can bitrate 125000
fi
echo -e "# run sick_line_guidance:\n# ip -details link show can0"
ip -details link show can0

# check can net device by sending 000#0x820A (can command reset communication to device 0x0A), should be answered by 70A#00 (boot message from device 0x0A)
# for ((n=0;n<=2;n++)) ; do
#   candump -ta -n 2 can0 &
#   cansend can0 000#820A
#   sleep 1
# done

# Start OLS20 simulation
# ./runsimu.bash

# start can logging
# printf "\033c" ; candump -ta can0 2>&1 | tee ~/.ros/log/candump.log
candump -ta can0 2>&1 | tee ~/.ros/log/candump.log &

# get/set values from object directory with canopen_chain_node service, f.e. Object 2001sub5 (sensorFlipped, UINT8, defaultvalue: 0x01)
# rosservice call /driver/get_object "{node: 'node1', object: '2001sub5', cached: false}"
# rosservice call /driver/set_object "{node: 'node1', object: '2001sub5', value: '0x01', cached: false}"

# Start ros driver canopen_chain_node, sick_line_guidance_node and sick_line_guidance_cloud_publisher

# echo -e "\n# run sick_line_guidance:\n# roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols10.yaml\n"
# roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols10.yaml 2>&1 | tee ~/.ros/log/sick_line_guidance_ols10.log # start OLS10 driver

echo -e "\n# run sick_line_guidance:\n# roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml\n"
roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml 2>&1 | tee ~/.ros/log/sick_line_guidance_ols20.log # start OLS20 driver

# echo -e "\n# run sick_line_guidance:\n# roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_mls.yaml\n"
# roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_mls.yaml 2>&1 | tee ~/.ros/log/sick_line_guidance_mls.log # start MLS driver

# read some object indices (f.e. 1001=ErrorRegister, 1018sub1=VendorID, 1018sub4=SerialNumber)
# rostopic echo -n 1 /node1_1001 & rostopic echo -n 1 /node1_1018sub1 & rostopic echo -n 1 /node1_1018sub4
# rosservice call /driver/get_object node1 1001sub  false
# rosservice call /driver/get_object node1 1018sub1 false
# rosservice call /driver/get_object node1 1018sub4 false

# Check errors and warnings in ros logfiles
killall candump
grep "\[ WARN\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*/sick*.log >> ~/.ros/log/ros_log_warnings.txt
grep "\[ERROR\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*/sick*.log >> ~/.ros/log/ros_log_errors.txt
cat ~/.ros/log/ros_log_warnings.txt
cat ~/.ros/log/ros_log_errors.txt

# Zip all logfiles
mkdir ./tmp
cp -rf ~/.ros/log ./tmp
now=$(date +"%Y%m%d_%H%M%S")
tar -cvzf ./$now-ros-logfiles.tgz ./tmp/log
rm -rf ./tmp

