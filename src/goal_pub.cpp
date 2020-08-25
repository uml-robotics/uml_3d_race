#include "ros/ros.h"
#include <string>
#include <std_srvs/Empty.h>

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

ros::Publisher goal_pub;
int counter;
float goal_x;
float goal_y;
float spawn_x;
float spawn_y;
float goal_tolerance;

bool callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  uml_3d_race::Goal goal;

  //Determines which goal needs to be published
  if(counter % 2 == 0)
  {
    //Set the goal to the original goal coords
    goal.x = goal_x;                  
    goal.y = goal_y;
    goal.tolerance = goal_tolerance;
  }
  else
  {
    //Set the goal to the original spawn coords
    goal.x = spawn_x;                  
    goal.y = spawn_y;
    goal.tolerance = goal_tolerance;
  }

  //Publish the goal
  goal_pub.publish(goal);

  //Increase count to alternate goals
  counter++;
  
  return true;
}

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;

  //Set default values
  counter = 0;
  goal_x = 0.0;
  goal_y = 0.0;
  spawn_x = 0.0;
  spawn_y = 0.0;
  goal_tolerance = 0.9;
  std::string topic = "goal";

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/goal_x",goal_x);
  n.getParam(ros::this_node::getName()+"/goal_y",goal_y);
  n.getParam(ros::this_node::getName()+"/spawn_x",spawn_x);
  n.getParam(ros::this_node::getName()+"/spawn_y",spawn_y);
  n.getParam(ros::this_node::getName()+"/goal_tolerance",goal_tolerance);
  n.getParam(ros::this_node::getName()+"/topic",topic);

  //Display node information on startup
  ROS_INFO("GOAL_A | Point: (%.2f,%.2f)\tGOAL_B | Point: (%.2f,%.2f)\tTolerance: %.2f\tTopic: %s",goal_x,goal_y,spawn_x,spawn_y,goal_tolerance,topic.c_str());

  //Create the publisher object
  goal_pub = n.advertise<uml_3d_race::Goal>(topic, 10, false);
  
  //Create the service callback
  ros::ServiceServer goal_server = n.advertiseService("get_new_goal", callback);

  //Keeps the node running and perform necessary updates
  ros::spin();

  return 0;
}
