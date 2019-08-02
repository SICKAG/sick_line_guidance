#!/bin/bash

# set environment, clear screen and logfiles
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi 
printf "\033c"
source ../../../../install/setup.bash
echo -e "\n# runsimu.bash: Stopping rosmaster and all rosnodes...\n# rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5"
rosnode kill -a ; sleep 5 ; killall roslaunch ; sleep 5 ; killall rosmaster ; sleep 5
rm -rf ~/.ros/log/*

# Initialize can net device, can0 by peak can adapter
# sudo ip link set can0 type can loopback on # loopback not supported
# sudo ip link set can0 up type can bitrate 125000 # set default bitrate 125000 bit/s

# Reset can0 net device
# sudo ip link set can0 down
# sleep 1
# sudo ip link set can0 up type can bitrate 125000
# sleep 1

# Set default bitrate 125000 bit/s
NUM_BITRATE_125000=$(ip -details link show can0 | grep "bitrate 125000" | wc -l)
if [ "$NUM_BITRATE_125000" = "0" ] ; then
  echo -e "# run ols/mls simulation:\n# sudo ip link set can0 up type can bitrate 125000"
  sudo ip link set can0 up type can bitrate 125000
fi
echo -e "# run ols/mls simulation:\n# ip -details link show can0"
ip -details link show can0
sleep 1

# Start can logging
# candump -ta can0 &
# candump -ta can0 2>&1 | tee ~/.ros/log/candump.log &

# Simulation with cangineberry: Upload firmware (BEDS slave) and beds-file CiA401_IO_Node3
# ./coiaupdater -p /dev/ttyS0 -f ../APPS/CANopenIA-BEDS/CgB_COIA_BEDS1.3_sec.bin # install firmware CANopen IA BEDS slave
# ./coiaupdater -p /dev/ttyS0 -d ../APPS/CANopenIA-BEDS/EDS/CiA401_IO_Node3.bin
# Simulation with cangineberry: Read/write objects 6401sub1 (LCP1), 6401sub2 (LCP2), 6401sub3 (LCP3), 6401sub4 (WidthLCP1), 6401sub5 (WidthLCP2), 6401sub6 (WidthLCP3)
# ./coia -p /dev/ttyS0 --read=0x6401,0x01
# ./coia -p /dev/ttyS0 --write=0x6401,0x01,2,0x1234
# ./coia -p /dev/ttyS0 --write=0x6401,0x01,2,0xABCD

# get/set values from object directory with canopen_chain_node service, f.e. Object 2001sub5 (sensorFlipped, UINT8, defaultvalue: 0x01)
# rosservice call /driver/get_object "{node: 'node1', object: '2001sub5', cached: false}"
# rosservice call /driver/set_object "{node: 'node1', object: '2001sub5', value: '0x01', cached: false}"

# Run simulation for OLS and MLS by yaml-file configuration
device_settings_map=( "OLS20:sick_line_guidance_ols20.yaml;120" "OLS10:sick_line_guidance_ols10.yaml;30" "MLS:sick_line_guidance_mls.yaml;30" )
for((device_cnt=0;device_cnt<=2;device_cnt++)) ; do
  device=${device_settings_map[$device_cnt]%%:*}
  settings_str=${device_settings_map[$device_cnt]##*:}
  IFS=';' settings_list=($settings_str)
  yaml_file=${settings_list[0]}
  run_seconds=${settings_list[1]}
  echo -e "runsimu.bash: settings for $device device: yaml_file $yaml_file, run for $run_seconds seconds."

  # Start can2ros converter
  # echo -e "\n# run ols/mls simulation:\n# roslaunch -v --screen sick_line_guidance sick_line_guidance_can2ros_node.launch\n"
  # roslaunch -v --screen sick_line_guidance sick_line_guidance_can2ros_node.launch &
  echo -e "\n# run ols/mls simulation:\n# roslaunch -v sick_line_guidance sick_line_guidance_can2ros_node.launch\n"
  roslaunch -v sick_line_guidance sick_line_guidance_can2ros_node.launch &
  sleep 5

  # Start ros2can converter
  # echo -e "\n# run ols/mls simulation:\n# roslaunch -v --screen sick_line_guidance sick_line_guidance_ros2can_node.launch\n"
  # roslaunch -v --screen sick_line_guidance sick_line_guidance_ros2can_node.launch &
  echo -e "\n# run ols/mls simulation:\n# roslaunch -v sick_line_guidance sick_line_guidance_ros2can_node.launch\n"
  roslaunch -v sick_line_guidance sick_line_guidance_ros2can_node.launch &
  sleep 5

  # Start OLS20 simulation
  echo -e "\n# run ols/mls simulation:\n# roslaunch -v sick_line_guidance sick_canopen_simu.launch device:=$device\n"
  roslaunch -v --screen sick_line_guidance sick_canopen_simu.launch device:=$device 2>&1 | tee ~/.ros/log/sick_canopen_simu_$device.log &
  # echo -e "\n# run ols/mls simulation:\n# roslaunch -v sick_line_guidance sick_canopen_simu.launch device:=$device\n"
  # roslaunch -v sick_line_guidance sick_canopen_simu.launch device:=$device &
  sleep 5

  # check can net device by sending 000#0x820A (can command reset communication to device 0x0A), should be answered by 70A#00 (boot message from device 0x0A)
  for ((n=0;n<1;n++)) ; do
    candump -ta -n 2 can0 &
    cansend can0 000#820A
    sleep 2
  done

  # Start ros driver for MLS or OLS, incl. canopen_chain_node, sick_line_guidance_node and sick_line_guidance_cloud_publisher
  echo -e "\n# run sick_line_guidance:\n# roslaunch -v sick_line_guidance sick_line_guidance.launch yaml:=$yaml_file\n"
  roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=$yaml_file 2>&1 | tee ~/.ros/log/$yaml_file.log &
  # echo -e "\n# run sick_line_guidance:\n# roslaunch -v sick_line_guidance sick_line_guidance.launch yaml:=$yaml_file\n"
  # roslaunch -v sick_line_guidance sick_line_guidance.launch yaml:=$yaml_file &

  # Run simulation for a while
  sleep $run_seconds

  # Exit simulation, shutdown all ros nodes
  echo -e "\n# runsimu.bash: Stopping rosmaster and all rosnodes...\n# rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5"
  rosnode kill -a ; sleep 5 ; killall rosmaster ; sleep 5

done 


# Check errors and warnings in ros logfiles
killall candump
echo -e "\n#\n# OLS/MLS simulation finished.\n#\n"
grep "\[ WARN\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*/sick*.log >> ~/.ros/log/ros_log_warnings.txt
grep "\[ERROR\]" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*/sick*.log >> ~/.ros/log/ros_log_errors.txt
grep "MeasurementVerificationStatistic" ~/.ros/log/*.log ~/.ros/log/*/ros*.log ~/.ros/log/*/sick*.log >> ~/.ros/log/simulation_statistic.txt
echo -e "\nSimulation warnings and errors:\n"
cat ~/.ros/log/ros_log_warnings.txt
cat ~/.ros/log/ros_log_errors.txt
echo -e "\nSimulation statistic:\n"
cat ~/.ros/log/simulation_statistic.txt

