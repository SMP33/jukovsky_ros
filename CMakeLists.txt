cmake_minimum_required(VERSION 2.8.3)
project(jukovsky_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

#set(PROJECT_BINARY_DIR ${CATKIN_PACKAGE_BIN_DESTINATION})


 add_message_files(
   FILES
juk_dji_gps_msg.msg
juk_dji_device_status_msg.msg
juk_control_dji_msg.msg
juk_position_data_msg.msg
juk_set_target_data_msg.msg
juk_dji_camera_control_msg.msg
juk_navigation_actions_msg.msg
juk_aruco_module_action.msg
juk_aruco_module_data.msg
reach_msg.msg
 )
 
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
 

add_subdirectory("src/core")
add_subdirectory("src/route_control")

#############
## Install ##
#############

#file(GLOB JUK_BINARY "juk_core/bin/*")
#install(FILES ${JUK_BINARY} DESTINATION  ${CATKIN_PACKAGE_BIN_DESTINATION})

