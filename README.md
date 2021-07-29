# kobuki_multi_robot ROS package

This package includes ROS launch files for starting up kobukis with their respective sensors (RealSense, Hokuyo or RPLIDAR). The required packages for each kobuki's NUC are assumed to be installed.

## How to build kobuki_multi_robot package
Copy this repository to your catkin workspace src folder:
 
```bash
git clone https://github.com/esauortiz/kobuki_multi_robot.git
```

## Usage in real environment
### Launch kobuki and its sensor (on kobuki)
Launches kobuki and its sensor

```bash
roslaunch kobuki_multi_robot kobuki.launch robot_name sensor
```
* Kobuki name is specified with ```robot_name``` parameter and can take the following values: ```kobuki_a```, ```kobuki_b```, ```kobuki_c```, ```kobuki_d``` and ```kobuki_e```.

* Sensor parameter specifies the ```sensor``` placed on the kobuki and can take the following values: ```realsense```, ```hokuyo04LX```, ```hokuyo20LX``` and ```rplidar```.

### Launch kobuki navigation (on kobuki)

Launches [navigation stack](http://wiki.ros.org/navigation) and ```goal listener``` node (subscribed to ```goal master publisher```, publishes goals for kobuki if there is some goals with ```ID = kobuki_id```)

	roslaunch kobuki_multi_robot kobuki_navigation.launch robot_name kobuki_id init_pose_x init_pose_y
	
* Kobuki name is specified with ```robot_name``` parameter and can take the following values: ```kobuki_a```, ```kobuki_b```, ```kobuki_c```, ```kobuki_d``` and ```kobuki_e```.

* Kobuki id is specified with ```kobuki_id``` parameter and can takes and integer value.

* [AMCL](http://wiki.ros.org/amcl) node requieres and initial kobuki pose specified with ```init_pose_x``` and ```init_pose_y```.