kobuki_multi_robot ROS package
=====================================================================
This package includes the ros launch files for starting up kobukis with their respective sensors (realsense, hokuyo or rplidar). 
The required packages for each kobuki's NUC are assumed to be installed.

How to build kobuki_multi_robot package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build kobuki_multi_robot

How to run kobuki_multi_robot package (real environment)
=====================================================================
Launch kobuki and its sensor (on kobuki)
------------------------------------------------------------
Launches kobuki and its sensor

	roslaunch kobuki_multi_robot kobuki.launch robot_name sensor

robot_name : {'kobuki_a', 'kobuki_b', 'kobuki_c', 'kobuki_d', 'kobuki_e'}

sensor : {'realsense', 'hokuyo04lx', 'hokuyo20lx', 'rplidar'}

Launch kobuki navigation (on kobuki)
------------------------------------------------------------
Launches navigation stack and goal slave SAP (subscribed to goal master publisher, publishes goals for kobuki if there is some goals with ID = kobuki_id)

	roslaunch kobuki_multi_robot kobuki_navigation.launch robot_name kobuki_id init_pose_x init_pose_y
	
robot_name : {'kobuki_a', 'kobuki_b', 'kobuki_c', 'kobuki_d', 'kobuki_e'}

kobuki_id : {1, 2, 3, 4, 5}

init_pose_x and init_pose_y : initial coordinates in map_frame

Launch test (on base station)
------------------------------------------------------------
Launches kobuki description, rviz and goal master publisher. Goal master publications are triggered by rostopic pub /update_global_goals std_msgs/Int8 "data: 1" (see globalgoals.txt file)

	roslaunch kobuki_multi_robot test.launch map_name

map_name : {'srvmap', 'willowgarage'}

How to run kobuki_multi_robot package (simulation)
=====================================================================
Launch gazebo with environment, kobuki and its sensor (on base station)
------------------------------------------------------------
Launches similation environment. The number of kobukis, their name and their initial position have to be specified in kobuki_gazebo.launch file

	roslaunch kobuki_multi_robot kobuki_gazebo.launch
	
Launch navigation stack for kobukis in gazebo (on base station)
------------------------------------------------------------
Launches navigation stack for each kobuki in similation. The number of kobukis, their name, their ID and their initial position have to be specified in kobuki_gazebo_navigation.launch file

	roslaunch kobuki_multi_robot kobuki_gazebo_navigation.launch
