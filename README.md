# kobuki_multi_robot ROS package

This package includes ROS launch files for starting up kobukis with their respective sensors (RealSense, Hokuyo or RPLIDAR). The required packages for each kobuki's NUC are assumed to be installed.

## How to build kobuki_multi_robot package
Copy this repository to your catkin workspace src folder:
 
```bash
git clone https://github.com/esauortiz/kobuki_multi_robot.git
```

Build catkin_ws at ~/catkin_ws directory:
 
```bash
catkin_make
```

------------------------------------------------------------
## Usage in real environment
### Launch kobuki and its sensor (on kobuki)
Launches kobuki and its sensor

```bash
roslaunch kobuki_multi_robot kobuki.launch robot_name sensor
```
* Kobuki name is specified with ```robot_name``` parameter and can take the following values: ```kobuki_a```, ```kobuki_b```, ```kobuki_c```, ```kobuki_d``` and ```kobuki_e```.

* Sensor parameter specifies the ```sensor``` placed on the kobuki and can take the following values: ```realsense```, ```hokuyo04LX```, ```hokuyo20LX``` and ```rplidar```.

This roslaunch file could be also run with the next .sh file placed in [start_and_nav](https://github.com/esauortiz/kobuki_multi_robot/tree/master/start_and_nav) folder.

```bash
./kobuki_start.sh
```

Each kobuki has environmental variables defined in the .bashrc file in order to complete robot_name and sensor arguments.

### Launch kobuki navigation (on kobuki)

Launches [navigation stack](http://wiki.ros.org/navigation) and ```goal listener``` node (subscribed to ```goal master publisher```, publishes goals for kobuki if there is some goals with ```ID = kobuki_id```)

	roslaunch kobuki_multi_robot kobuki_navigation.launch robot_name kobuki_id mission
	
* Kobuki name is specified with ```robot_name``` parameter and can take the following values: ```kobuki_a```, ```kobuki_b```, ```kobuki_c```, ```kobuki_d``` and ```kobuki_e```.

* Kobuki id is specified with ```kobuki_id``` parameter and can takes and integer value.

* [AMCL](http://wiki.ros.org/amcl) node requires and initial kobuki pose specified with ```init_pose_x```, ```init_pose_y``` and ```init_pose_a```. This arguments are defined in [param/missions](https://github.com/esauortiz/kobuki_multi_robot/tree/master/param/missions) folder.

This roslaunch file could be also run with the next .sh file place in [start_and_nav](https://github.com/esauortiz/kobuki_multi_robot/tree/master/start_and_nav) folder.

```bash
./kobuki_navigation.sh --mission default
```

Each kobuki has environmental variables defined in the .bashrc file in order to complete robot_name and sensor arguments. The mission (option -m|--mission) has to be specified in order to provide an initial pose for each kobuki.

### Launch test (on base station)
Launches ```kobuki description```, ```rviz``` and ```goal master publisher```. Goal master publications are triggered by ```rostopic pub /update_global_goals std_msgs/Int8 "data: 1"``` (see README file in [src](https://github.com/esauortiz/kobuki_multi_robot/tree/master/src) folder)

```bash
roslaunch kobuki_multi_robot base_station.launch map_name
```
See available maps in [maps](https://github.com/esauortiz/kobuki_multi_robot/tree/master/maps) folder.

------------------------------------------------------------
## Usage in simulation (OUTDATED)

### Launch gazebo with environment, kobuki and its sensor (on base station)
Launches similation environment. The number of kobukis, their name and their initial position have to be specified in ```kobuki_gazebo.launch``` file

```bash
roslaunch kobuki_multi_robot kobuki_gazebo.launch
```

### Launch navigation stack for kobukis in gazebo (on base station)
Launches [navigation stack](http://wiki.ros.org/navigation) for each kobuki in similation. The number of kobukis, their name, their ID and their initial position have to be specified in ```kobuki_gazebo_navigation.launch``` file

```bash
roslaunch kobuki_multi_robot kobuki_gazebo_navigation.launch
```

------------------------------------------------------------
## Some usage considerations
* This package provides two ```bash``` files for each kobuki called ```kobuki_start.sh``` and ```kobuki_navigation.sh``` to make deployment of the kobukis easier. Each kobuki has to access its respective folder in the main folder ```start_and_nav```. Running ```source permissions.sh``` gives both start and navigation files executing permissions.
* ```kobuki_start.sh``` has to be executed before ```kobuki_navigation.sh```
* ```kobuki_d```has a RPLIDAR mounted. This sensor provides a range of 360 degrees but the scans placed in the 'back' of the kobuki have been 'removed' by means of [laser filter](http://wiki.ros.org/laser_filters) node. However, the messages published in the topic ```/kobuki_d/scan``` contain 360 degrees and the 'removed' scans have values of ```range_max + 1```, which is assumed to be an error case.

## Troubleshooting

- The first time the navigation stack nodes are launched the cost map does not update properly and will always be 'empty'
	- Solution: Run ```kobuki_navigation.sh```, let everything be initialized until you get to the ```odom received!``` message in your terminal, cancel the execution and run the same file again.  In this way the cost map is updated correctly and the navigation stack runs properly.