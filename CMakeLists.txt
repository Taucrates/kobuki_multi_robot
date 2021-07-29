cmake_minimum_required(VERSION 3.0.2)
project(kobuki_multi_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2_ros
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   global_goal.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

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
#  INCLUDE_DIRS include
#  LIBRARIES kobuki_multi_robot
  CATKIN_DEPENDS roscpp roslaunch rospy std_msgs geometry_msgs message_runtime tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# nodes
add_executable(multi_robot_master src/multi_robot_master.cpp)
target_link_libraries(multi_robot_master ${catkin_LIBRARIES})

add_executable(multi_robot_slave src/multi_robot_slave.cpp)
target_link_libraries(multi_robot_slave ${catkin_LIBRARIES})

# deps
add_dependencies(multi_robot_master kobuki_multi_robot_generate_messages_cpp)
add_dependencies(multi_robot_slave kobuki_multi_robot_generate_messages_cpp)

