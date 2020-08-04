#include "ros/ros.h"
#include <string>
#include <gazebo_msgs/ContactsState.h>

void CollisionCallback(gazebo_msgs::ContactsState collision)
{
    if(!collision.states.empty())
    {
      ROS_INFO("Collision at x:%f y:%f", collision.states[0].contact_positions[0].x, collision.states[0].contact_positions[0].y);
    }
}

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "collision_subscriber");
  ros::NodeHandle n;

  //Set default values
  std::string topic = "pioneer/bumper_contact";

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/topic",topic);

  //Create the subscriber object
  ros::Subscriber collision_sub = n.subscribe(topic, 1, CollisionCallback);

  ros::spin();

  return 0;
}
