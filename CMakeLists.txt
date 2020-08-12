cmake_minimum_required(VERSION 2.8.3)
project(sick_line_guidance)

##
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall -Wno-reorder -Wno-sign-compare -Wno-unused-local-typedefs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  can_msgs 
  canopen_chain_node
  message_generation
  random_numbers
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MLS_Measurement.msg
  OLS_Measurement.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  can_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sick_line_guidance_lib
  CATKIN_DEPENDS can_msgs canopen_chain_node message_runtime random_numbers roscpp rospy sensor_msgs std_msgs
  DEPENDS TinyXML # system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/sick_line_guidance_can_subscriber.cpp
  src/sick_line_guidance_can_cia401_subscriber.cpp
  src/sick_line_guidance_can_mls_subscriber.cpp
  src/sick_line_guidance_can_ols_subscriber.cpp
  src/sick_line_guidance_canopen_chain.cpp
  src/sick_line_guidance_cloud_converter.cpp
  src/sick_line_guidance_diagnostic.cpp
  src/sick_line_guidance_eds_util.cpp
  src/sick_line_guidance_msg_util.cpp
  src/sick_canopen_simu_canstate.cpp
  src/sick_canopen_simu_device.cpp
  src/sick_canopen_simu_verify.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node            src/sick_line_guidance_node.cpp)
add_executable(${PROJECT_NAME}_can_chain_node  src/sick_line_guidance_can_chain_node.cpp)
add_executable(${PROJECT_NAME}_cloud_publisher src/sick_line_guidance_cloud_publisher.cpp)
add_executable(${PROJECT_NAME}_can2ros_node    src/sick_line_guidance_can2ros_node.cpp)
add_executable(${PROJECT_NAME}_ros2can_node    src/sick_line_guidance_ros2can_node.cpp)
add_executable(sick_canopen_simu_node          src/sick_canopen_simu_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  random_numbers
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_can_chain_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_cloud_publisher
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_can2ros_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_ros2can_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(sick_canopen_simu_node
  random_numbers
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  canopen_ros_chain
  random_numbers
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_can_chain_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_cloud_publisher
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_can2ros_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_ros2can_node
  canopen_ros_chain
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(sick_canopen_simu_node
  canopen_ros_chain
  random_numbers
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_can_chain_node ${PROJECT_NAME}_cloud_publisher ${PROJECT_NAME}_can2ros_node ${PROJECT_NAME}_ros2can_node sick_canopen_simu_node ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/sick_line_guidance.launch
  launch/sick_line_guidance_can2ros_node.launch
  launch/sick_line_guidance_ros2can_node.launch
  launch/sick_canopen_simu.launch
  mls/SICK-MLS.eds
  mls/sick_line_guidance_mls.yaml
  ols/SICK_OLS10_CO.eds
  ols/SICK_OLS20.eds
  ols/SICK_OLS20_CO.eds
  ols/sick_line_guidance_ols10.yaml
  ols/sick_line_guidance_ols20.yaml
  test/cfg/sick_canopen_simu_cfg.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sick_line_guidance.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


if( TURTLEBOT_DEMO )
  add_subdirectory( turtlebotDemo/agc_radar )
  add_subdirectory( turtlebotDemo/custom_messages )
  add_subdirectory( turtlebotDemo/gpio_handling )
  add_subdirectory( turtlebotDemo/lidar_obstacle_detection )
  add_subdirectory( turtlebotDemo/sick_line_guidance_demo )
endif( TURTLEBOT_DEMO )
