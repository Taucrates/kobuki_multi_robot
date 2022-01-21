cmake_minimum_required(VERSION 3.0.2)
project(kobuki_multi_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2_ros
  gazebo_msgs
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
   gazebo_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES kobuki_multi_robot
  CATKIN_DEPENDS roscpp roslaunch rospy std_msgs geometry_msgs message_runtime tf2_ros gazebo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# nodes
add_executable(multi_robot_master src/multi_robot_master.cpp)
target_link_libraries(multi_robot_master ${catkin_LIBRARIES})

add_executable(multi_robot_master_v2 src/multi_robot_master_v2.cpp)
target_link_libraries(multi_robot_master_v2 ${catkin_LIBRARIES})

add_executable(multi_robot_slave src/multi_robot_slave.cpp)
target_link_libraries(multi_robot_slave ${catkin_LIBRARIES})

add_executable(model_state_setter src/model_state_setter.cpp)
target_link_libraries(model_state_setter ${catkin_LIBRARIES})

# deps
add_dependencies(multi_robot_master kobuki_multi_robot_generate_messages_cpp)
add_dependencies(multi_robot_master_v2 kobuki_multi_robot_generate_messages_cpp)
add_dependencies(multi_robot_slave kobuki_multi_robot_generate_messages_cpp)
