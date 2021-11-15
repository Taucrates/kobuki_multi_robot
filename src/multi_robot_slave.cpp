/**
 * @Author: Esau Ortiz Toribio
 * @File: multi_robot_slave.cpp
 * @Description: Multi robot slave. Subscribes to the my_global_goals topic and reads the identifier
 *               to verify if it corresponds to a command for the kobuki on which this node runs
 *
 *               In addition this node calls "clear_costmaps" service to avoid bad local planner behaviour
 * @Date: March 2021
 */

#include "ros/ros.h"
#include <kobuki_multi_robot/global_goal.h> // global_goal_msg
#include "geometry_msgs/PoseStamped.h"
#include <std_srvs/Empty.h> // needed for clearCLient service

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // Publisher
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    // Subscriber
    sub_ = n_.subscribe("/my_global_goals", 10, &SubscribeAndPublish::callback, this);
    // setting kobuki_id
    n_.getParam("multi_robot_slave/kobuki_id", kobuki_id_);
  }

  // Verify kobuki_id and publish if required  
  void callback(const kobuki_multi_robot::global_goal::ConstPtr & global_goal)
  {
    if(global_goal->id == kobuki_id_){
      pub_.publish(global_goal->pose_stamped);
    }
  }

public:
  ros::NodeHandle n_; 
private:
  //ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int kobuki_id_;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_robot_slave");

  // SAP object
  SubscribeAndPublish SAPObject;
 
  // Loop rate
  ros::Rate loop_rate(10);

  // loop counter
  int loop_counter = 0;

  while (ros::ok())
  {
    //ROS_INFO("Checking if there is new goals for this kobuki");
    // clear cost map service
    ros::service::waitForService("move_base/clear_costmaps");
    ros::ServiceClient clearClient = SAPObject.n_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    std_srvs::Empty srv;
    if(loop_counter == 300){
      //ROS_INFO("Checking if there is new goals for this kobuki");
      // Posible mejora del criterio para elegir la frecuencia
      // de llamada la servicio: comprobar si la velocidad en x es != 0
      clearClient.call(srv);
      loop_counter = 0;
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    loop_counter++;
  }

  return 0;
}

