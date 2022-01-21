/**
 * @Author: Esau Ortiz Toribio
 * @File: multi_robot_master.cpp
 * @Description: Multi robot master. Reads a list of global goals file and publishes them 
 *               everytime a signal (new_global_goals) arrives from `\update_global_goals` topic.
 * @Date: March 2021
 */

#include "ros/ros.h"
#include "std_msgs/Int8.h" // update_global_goals_msg
#include <kobuki_multi_robot/global_goal.h> // global_goal_msg
#include <fstream>
#include <vector>

#include <tf2/LinearMath/Quaternion.h> // from rpy to quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h> // tf global_goal from world to map
#include <tf2_ros/buffer.h> // tf global_goal from world to map

bool new_global_goals = false;

// Callbacks
void updateGlobalGoalsCallback(const std_msgs::Int8::ConstPtr & update_global_goals_msg);

// Auxiliar methods
void getGlobalGoals(std::vector<std::vector<float>>* global_goals, const std::string& global_goals_path_file);
void publishGlobalGoal(const std::vector<float>& global_goal_vect, ros::Publisher* global_goal_pub, tf2_ros::Buffer* tf_buffer);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_robot_master");
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher global_goal_pub = nh.advertise<kobuki_multi_robot::global_goal>("/my_global_goals", 10);

  // Subscribers
  ros::Subscriber goal_status_sub = nh.subscribe("/update_global_goals", 10, updateGlobalGoalsCallback);

  // Loop rate
  ros::Rate loop_rate(10);

  // global_goals vector
  std::vector<std::vector<float>> global_goals;

  // read path to global_goals file
  std::string global_goals_path_file;
  nh.getParam("/multi_robot_master/global_goals_path_file", global_goals_path_file);

  // TF (map to world)
  tf2_ros::Buffer tf_buffer;

  while (ros::ok())
  {

    if(new_global_goals == true){
        getGlobalGoals(&global_goals, global_goals_path_file);

        // publish global goals
        for(int i = 0; i < global_goals.size(); i++){
            publishGlobalGoal(global_goals[i], &global_goal_pub, &tf_buffer);
        }

        ROS_INFO("Global goals have been published");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/*******************************************************************
goalStatusCallback method
This callback dicates if the global goals have been updated         
*******************************************************************/

void updateGlobalGoalsCallback(const std_msgs::Int8::ConstPtr & update_global_goals_msg){
    if(update_global_goals_msg->data == 1){
        new_global_goals = true;
    }else{
        new_global_goals = false;
    }
}

/*******************************************************************
getGlobalGoals method
Reads global goals file
*******************************************************************/
void getGlobalGoals(std::vector<std::vector<float>>* global_goals, const std::string& global_goals_path_file){

    // clearing global_goals vector
    global_goals->clear();
    // reading global_goals file
    std::ifstream infile(global_goals_path_file);
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        float id, x, y, z, roll, pitch, yaw;
        if (!(iss >> id >> x >> y >> z >> roll >> pitch >> yaw)) { break; } // error

        // building global_goals vector
        std::vector<float> global_goal = {id, x, y, z, roll, pitch, yaw};
        global_goals->push_back(global_goal);
    }
    new_global_goals = false;
}

/*******************************************************************
publishGlobalGoal method
Publishes a global goal
*******************************************************************/
void publishGlobalGoal(const std::vector<float>& global_goal_vect, ros::Publisher* global_goal_pub, tf2_ros::Buffer* tf_buffer){

    // SCDs
    std::string source_frame = "map";
    //std::string target_frame = "map";

    // msg to be published
    kobuki_multi_robot::global_goal global_goal;
    
    global_goal.id = global_goal_vect[0];

    /*switch(global_goal.id){
        case 1:
            source_frame = "kobuki_a/map";
        break;
        case 2:
            source_frame = "kobuki_b/map";
        break;
        case 3:
            source_frame = "kobuki_c/map";
        break;
        case 4:
            source_frame = "kobuki_d/map";
        break;
        case 5:
            source_frame = "kobuki_e/map";
        break;
    }*/
    //global_goal.pose_stamped.header.seq = ros::Time::now();
    global_goal.pose_stamped.header.stamp = ros::Time::now();
    global_goal.pose_stamped.header.frame_id = source_frame;    // global goal read from file is referenced to map frame 
    global_goal.pose_stamped.pose.position.x = global_goal_vect[1];
    global_goal.pose_stamped.pose.position.y = global_goal_vect[2];
    global_goal.pose_stamped.pose.position.z = global_goal_vect[3];

    // from roll pitch yaw to x,y,z,w
    tf2::Quaternion quaternion;
    quaternion.setRPY( global_goal_vect[4], global_goal_vect[5], global_goal_vect[6]);  // Create this quaternion from roll/pitch/yaw (in radians)
    quaternion = quaternion.normalize();
    global_goal.pose_stamped.pose.orientation = tf2::toMsg(quaternion);

    // apply TF 
    // TF is commented because we assume that global goals are referenced to map frame
    // otherwise, uncomment this lines to apply TF
    /*
    geometry_msgs::TransformStamped world_to_map;
    world_to_map = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0), ros::Duration(1.0) ); // target: map, source: world
    tf2::doTransform(global_goal.pose_stamped, global_goal.pose_stamped, world_to_map); 
    */
   
    global_goal_pub->publish(global_goal);
}
