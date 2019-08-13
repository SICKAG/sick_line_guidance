# TurtleBot demonstration

Demonstration of SICK line guidance demonstration with TurtleBot

## Build and install

To build and install a TurtleBot demonstration (sick_line_guidance_demo), follow the description [doc/build_install.md](doc/build_install.md)

## Build and run simulation

To build and run the sick_line_guidance_demo simulation, run the following commands:

```console
cd ~/catkin_ws/src
git clone https://github.com/SICKAG/sick_line_guidance.git
cd ./sick_line_guidance/turtlebotDemo/test/scripts
# ./gitCloneInstall.bash # install all packages for sick_line_guidance_demo (required once)
./makeall.bash # build everything for sick_line_guidance_demo
./runsimu.bash # run simulation of sick_line_guidance_demo
```

Note: ``` catkin_make ``` resp. ``` catkin_make install ``` will only build the SICK line guidance ros driver. 
To build the TurtleBot demonstration, an additional option ``` --cmake-args -DTURTLEBOT_DEMO="ON" ``` is required:

```console
cd ~/catkin_ws
catkin_make install --cmake-args -DTURTLEBOT_DEMO="ON"
source ./install/setup.bash
```

TurtleBot packages are not required unless this option is enabled (``` --cmake-args -DTURTLEBOT_DEMO="ON" ```).

## Setup TurtleBot

To setup the TurtleBot for sick_line_guidance_demo, follow the description [doc/setup_turtlebot.md](doc/setup_turtlebot.md)


## Run TurtleBot demonstration

To run the SICK line guidance demonstration with a TurtleBot, run the following commands:

```console
cd ~/catkin_ws/src/sick_line_guidance/turtlebotDemo/test/scripts
./runturtlebot.bash
```
