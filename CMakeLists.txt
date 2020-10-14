cmake_minimum_required(VERSION 3.0.2)
project(AKROS_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  motor_cmd_single.msg
  motor_reply_single.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Initialize.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AKROS_bridge
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(motor_value_converter    src/AK80_6.cpp src/basic_op.cpp src/motor_value_converter.cpp)
add_executable(motor_cmd_test_pub       src/test_pub_node/motor_cmd_test_pub.cpp)
add_executable(motor_can_reply_test_pub src/test_pub_node/motor_can_reply_test_pub.cpp)

target_link_libraries(motor_value_converter    ${catkin_LIBRARIES})
target_link_libraries(motor_cmd_test_pub       ${catkin_LIBRARIES})
target_link_libraries(motor_can_reply_test_pub ${catkin_LIBRARIES})
