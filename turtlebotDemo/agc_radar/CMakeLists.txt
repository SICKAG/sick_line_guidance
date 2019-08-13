cmake_minimum_required(VERSION 2.8.3)
project(agc_radar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   agc_radar_msg.msg
 )

 add_service_files(
   FILES
   agc_radar_config.srv
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

#catkin_package(
#  #INCLUDE_DIRS include
#  #LIBRARIES agc_radar
#  CATKIN_DEPENDS message_generation roscpp std_msgs message_runtime gpio_handling
#  #DEPENDS system_lib
#)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  # $HOME/catkin_ws/devel/include
)

add_executable(agc_radar src/agc_radar.cpp)
add_dependencies(agc_radar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agc_radar ${catkin_LIBRARIES})
