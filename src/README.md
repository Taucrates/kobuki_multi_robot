Master and slave nodes to publish goals
=====================================================================
The master node reads a list of global goals. Each global goal is composed of an identifier (id) and a stamped pose (pose_stamped). The slave node is subscribed to the global goals topic and checks if any global goal have the identifier of the kobuki where the slave node is running. If a global goal is associated with the kobuki identifier this global target is published in the kobuki global target topic /kobuki_id/move_base_simple/goal.

Usage
=====================================================================
    1) multi_robot_master should be running in the base station. Path to the global goals file should be setted (is recieved as a rosparam)
    2) multi_robot_slave should be running in each kobuki. kobuki_id should be setted as an integer (rosparam).
    3) multi_robot_master is suscribed to /update_global_goals and publishes all the global goals in file if receives a std_msgs/Int8 "data: 1".
