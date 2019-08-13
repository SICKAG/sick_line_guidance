cmake_minimum_required(VERSION 2.8.3)
project(lidar_obstacle_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

 add_message_files(
   FILES
   obstacle_detector_msg.msg
 )

 add_service_files(
   FILES
   obstacle_detector_config.srv
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

#catkin_package(
#  #INCLUDE_DIRS include
#  LIBRARIES lidar_obstacle_detection
#  CATKIN_DEPENDS message_generation roscpp std_msgs message_runtime
#  #DEPENDS system_lib
#)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})

add_executable(obstacle_detector_config src/obstacle_detector_config.cpp)
add_dependencies(obstacle_detector_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_detector_config ${catkin_LIBRARIES})
