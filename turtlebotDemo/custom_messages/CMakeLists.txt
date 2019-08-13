cmake_minimum_required(VERSION 2.8.3)
project(custom_messages)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs 
  roscpp
  )

 add_message_files(
   FILES
   gpio.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

#catkin_package(
#LIBRARIES custom_messages
#CATKIN_DEPENDS std_msgs roscpp
#)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

