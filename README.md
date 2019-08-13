# sick_line_guidance

SICK Line Guidance ROS Support

# Introduction

The aim of this project is to provide an ROS connection for lane guidance sensors of the OLS10, OLS20 and MLS family.

Users should regularly inform themselves about updates to this driver (best subscribe under "Watch").

# Supported Hardware

SICK optical and magnetical line sensors OLS10, OLS20 and MLS.

| Product Family  | Product Information and Manuals |
| --- | --- |
| OLS10 | https://www.sick.com/ols10 |
| OLS20 | https://www.sick.com/ols20 |
| MLS   | https://www.sick.com/mls |

# Installation and setup

## Setup CAN hardware and driver

&nbsp; 1. Install can-utils and socketcan:

```bash
sudo apt-get install can-utils
sudo modprobe can
sudo modprobe vcan
sudo modprobe slcan
```
Details can be found in the following links: https://wiki.linklayer.com/index.php/SocketCAN , https://gribot.org/installing-socketcan/ , 
https://www.kernel.org/doc/Documentation/networking/can.txt

&nbsp; 2. Install linux driver for your CAN hardware:

ROS support for sick line guidance sensors has been developed and tested using PEAK CAN adapter for the CAN communication.
Unless you're using other CAN hardware or driver, we recommend installation of pcan usb adapter by following the installation
instructions on [doc/pcan-linux-installation.md](doc/pcan-linux-installation.md)

## Installation from Source

Run the following script to install sick_line_guidance including all dependancies and packages required:

```bash
source /opt/ros/melodic/setup.bash # currently ros distro melodic is supported
cd ~ # or change to your project path
mkdir -p catkin_ws/src/
cd catkin_ws/src/
git clone https://github.com/SICKAG/sick_line_guidance.git
git clone https://github.com/ros-industrial/ros_canopen.git
git clone https://github.com/CANopenNode/CANopenSocket.git
git clone https://github.com/linux-can/can-utils.git
git clone https://github.com/ros-planning/random_numbers.git
cd ..
sudo apt-get install libtinyxml-dev
sudo apt-get install libmuparser-dev
catkin_make install
source ./install/setup.bash
```

# Configuration

If not done before, you have to set the CAN bitrate, f.e. to 125 kbit/s (default bitrate for OLS and MLS):
```bash
sudo ip link set can0 up type can bitrate 125000
ip -details link show can0 
```

The can node id of the OLS or MLS device is configured in a yaml-file:
- catkin_ws/src/sick_line_guidance/ols/sick_line_guidance_ols10.yaml for OLS10 devices
- catkin_ws/src/sick_line_guidance/ols/sick_line_guidance_ols20.yaml for OLS20 devices
- catkin_ws/src/sick_line_guidance/mls/sick_line_guidance_mls.yaml for MLS devices

The default can node id is 0x0A. To use a different can node id, set entry "id: 0x0A" to the appropriate value in the yaml-file:
```bash
  node1:
    id: 0x0A # CAN-Node-ID of can device, default: Node-ID 10=0x0A for OLS and MLS
```
Install the new configuration with 
```bash
cd catkin_ws
catkin_make install
source ./install/setup.bash
```
after modifications.

See https://github.com/SICKAG/sick_line_guidance/tree/master/doc/sick_line_guidance_configuration.md for details about the 
sick_line_guidance driver and sensor configuration.    

# Starting

To start the driver for MLS or OLS, the ros package sick_line_guidance and its launch-file has to be started by roslaunch:

```bash
cd ~/catkin_ws # change working directory to the project path
source ./install/setup.bash # set environment
roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols10.yaml # start OLS10 driver
roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml # start OLS20 driver
roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_mls.yaml   # start MLS driver
```

After successful start, you can observe the sensor measurement data in a shell by subscribing to ros topics "/ols", "/mls" and "/cloud":

```bash
source ./install/setup.bash
rostopic list 
rostopic echo /mls
rostopic echo /ols
rostopic echo /cloud
```

or you can plot the sensor positions by 

```bash
source ./install/setup.bash
rqt_plot /mls/position[0] /mls/position[1] /mls/position[2] # plot mls positions
rqt_plot /ols/position[0] /ols/position[1] /ols/position[2] # plot ols positions
```

You can visualize the data by starting rviz, subscribe to topic "/cloud" (PointCloud2) and select "ols_frame" or "mls_frame":

```bash
source ./install/setup.bash
rosrun rviz rviz 
```

# Troubleshooting and diagnostics

All measurements are published continously on ros topic "/mls" resp. "/ols". In addition, the current status (ok or error) 
and (in case of errors) an error code is published on topic "/diagnostics". Messages on these topics can be views by
```bash
rostopic echo /mls
rostopic echo /ols
rostopic echo /diagnostics
```
The following error codes are defined in header file sick_line_guidance_diagnostic.h:
```bash
  /*
   * enum DIAGNOSTIC_STATUS enumerates the possible status values of diagnostic messages.
   * Higher values mean more severe failures.
   */
  typedef enum DIAGNOSTIC_STATUS_ENUM
  {
    OK,                       // status okay, no errors
    EXIT,                     // sick_line_guidance exiting
    NO_LINE_DETECTED,         // device signaled "no line detected"
    ERROR_STATUS,             // device signaled an error (error flag set in TPDO)
    SDO_COMMUNICATION_ERROR,  // error in SDO query, timeout on receiving SDO response
    CAN_COMMUNICATION_ERROR,  // can communication error, shutdown and reset communication
    CONFIGURATION_ERROR,      // invalid configuration, check configuration files
    INITIALIZATION_ERROR,     // initialization of CAN driver failed
    INTERNAL_ERROR            // internal error, should never happen
  } DIAGNOSTIC_STATUS;
```

All data transmitted on the CAN bus can be displayed by candump:

```bash
candump -ta can0
```

In case of a successful installation, heartbeats and PDO messages should be visible.
Example output (can master requests object 1018sub4 from can node 8, device responds with its serial number 0x13015015):
```bash
$ candump -ta can0
(1549455524.265601)  can0  77F   [1]  05                      # heartbeat
(1549455524.294836)  can0  608   [8]  40 18 10 04 00 00 00 00 # SDO request (0x600+nodeid), 8 byte (0x40) data, object 0x101804
(1549455524.301181)  can0  588   [8]  43 18 10 04 15 50 01 13 # SDO response (0x580+nodeid), 4 byte (0x43) value, object 0x101804, value 0x13015015
```

Values in the object dictionary of the CAN device can be viewed by

```bash
# rosservice call /driver/get_object node1 <object_index> <cached>
rosservice call /driver/get_object node1 1018sub4 false # query serial number 
```
Example output:
```bash
$ rosservice call /driver/get_object node1 1018sub4 false # query serial number
success: True
message: ''
value: "318853141"
```

The error register of a can device (object index 0x1001) and its pdo mapped objects are published on ros topic "node1_<object_index>"
and can be printed by rostopic:

```bash
# rostopic echo -n 1 /node1_<object_index> # print object <object_index> of can device
rostopic echo -n 1 /node1_1001 # print error register 0x1001 of can device
```
Example output:
```bash
$ rostopic echo -n 1 /node1_1001
data: 0
```

For test purposes or in case of hardware problems, cansend can be used to send CAN messages. Example:

```bash
cansend can0 000#820A # NMT message to can device 0x0A: 0x82, reset communication
cansend can0 60A#4F01100000000000 # PDO request: read error register 1001
```

## Simulation and testing

A software simulation is available for test purposes. This simulation generates synthetical can frames from an input xml-file, and verifies the measurement messages published by
the sick_line_guidance driver. By sending specified can frames, the complete processing chain of the ros driver can be verified by comparing the actual measurement messages 
with the expected results. See https://github.com/SICKAG/sick_line_guidance/tree/master/doc/sick_canopen_simu.md for further details.

## TurtleBot demonstration

A demonstration of SICK line guidance with a TurtleBot robot is included in folder sick_line_guidance/turtlebotDemo. 
Please see [turtlebotDemo/README.md](turtlebotDemo/README.md) for further details.

## FAQ

### "Network is down"

:question: Question: 
```bash
candump -ta can0
```
gives the result
```bash
read: Network is down
```

:white_check_mark: Answer: 
(Re-)start can interface by the following commands:
```bash
sudo ip link set can0 type can
sudo ip link set can0 up type can bitrate 125000 # configure the CAN bitrate, f.e. 125000 bit/s
```

### "candump gives no answer"

:question: Question: 
```bash
candump -ta can0
```
gives no results.

:white_check_mark: Answer: 
Check the baud rate of your device. For a brand new OLS10 this could be 250000 Baud. 
For OLS10 please check the baud rate setting by using the device panel (read operation manual of your device).

### "device or resource busy"

:question: Question: 
```bash
sudo ip link set can0 up type can bitrate 125000
```
gives the result:
```bash
RTNETLINK answers: Device or resource busy
```

:white_check_mark: Answer: 
Check the baud rate of your device. For a brand new OLS10 this could be 250000 Baud. 
For OLS10 please check the baud rate setting by using the device panel (read operation manual of your device).
After checking (and changing) the baud rate unplug and replug the usb connector.


### "Device 'can0' does not exist"

:question: Question: After start, the message
```bash
Device "can0" does not exist.
```
is displayed.

:white_check_mark: Answer: 
- Check power supply
- Unplug, replug and restart PEAK-USB-Adapter
- If you use a PEAK-USB-Adapter and the error message still displays, re-install the PCAN-driver. 
PCAN driver can be overwritten by a default can driver due to system updates ("mainline drivers removed and blacklisted in /etc/modprobe.d/blacklist-peak.conf"). 
In this case, the PCAN driver must be re-installed. See the quick installation guide https://github.com/SICKAG/sick_line_guidance/tree/master/doc/pcan-linux-installation.md

### "Configuration changes do not take effect"

:question: Question: After editing configuration files, the configuration changes do not take effect.

:white_check_mark: Answer: Modified configuration files have to be installed by
```bash
cd catkin_ws
catkin_make install
source ./install/setup.bash
```
Restart the driver for MLS or OLS. To avoid potential errors due to previous ros nodes or processes still running, 
you might kill all ros nodes and the ros core by 
```bash
rosnode kill -a    # kill all ros nodes
killall rosmaster  # kill ros core
```

Please note, that this kills all ros processes, not just those required for sick_line_guidance.
