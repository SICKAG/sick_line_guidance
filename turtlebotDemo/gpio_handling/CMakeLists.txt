cmake_minimum_required(VERSION 2.8.3)
project(gpio_handling)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  gpio_set_config.srv
  gpio_get_config.srv
  gpio_set_pin.srv
  gpio_get_pin.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  custom_messages
 )

#catkin_package(
#  # INCLUDE_DIRS include
#  LIBRARIES gpio_handling
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#)

include_directories( ${catkin_INCLUDE_DIRS} )

# Service Server
add_executable(gpio_handler src/gpio_handler.cpp)
add_dependencies(gpio_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpio_handler ${catkin_LIBRARIES} )

install(TARGETS gpio_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
