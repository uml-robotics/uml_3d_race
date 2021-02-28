#include "ros/ros.h"
#include <string>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

std::string model_urdf;

float x_pos_1 = 0.0;
float y_pos_1 = 0.0;
float z_pos_1 = 0.0;
float theta_1 = 0.0;

float x_pos_2 = 0.0;
float y_pos_2 = 0.0;
float z_pos_2 = 0.0;
float theta_2 = 0.0;

bool side_toggle = false;

ros::ServiceClient spawner_service;
ros::ServiceClient delete_service;

void spawn_object(std::string object_name, float x, float y, float z, float theta)
{
  //Set object parameters
  gazebo_msgs::SpawnModel spawner_msg;
  spawner_msg.request.model_name = object_name;
  spawner_msg.request.model_xml = model_urdf;
  spawner_msg.request.robot_namespace = "";

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion angle;
  angle.setRPY(0, 0, theta);
  pose.orientation.z = angle.getZ();
  pose.orientation.w = angle.getW();

  spawner_msg.request.initial_pose = pose;
  spawner_msg.request.reference_frame = "";

  //Call the spawn service
  if(!spawner_service.call(spawner_msg))
  {
    ROS_WARN("Failed to spawn %s object", object_name.c_str());
  }
}

void delete_object(std::string object_name)
{
  //Set the name of the object that is getting removed
  gazebo_msgs::DeleteModel remover_msg;
  remover_msg.request.model_name = object_name;

  //Call the delete object service
  if(!delete_service.call(remover_msg))
  {
    ROS_WARN("Failed to delete %s object", object_name.c_str());
  }
}

void state_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &state)
{
    if (!state->status_list.empty())
    {
        if (state->status_list.back().status == state->status_list.back().ABORTED || state->status_list.back().status == state->status_list.back().REJECTED 
            || state->status_list.back().status == state->status_list.back().SUCCEEDED)
        {
            //Switch the objects location
            if(side_toggle)
            {
              spawn_object("dynamic_obstacle_1", x_pos_1, y_pos_1, z_pos_1, theta_1);
              delete_object("dynamic_obstacle_2");
            }
            else
            {
              spawn_object("dynamic_obstacle_2", x_pos_2, y_pos_2, z_pos_2, theta_2);
              delete_object("dynamic_obstacle_1");
            }

          //Toggle the side to spawn the next object
          side_toggle = !side_toggle;
        }
    }
}

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "dynamic_obstacle_spawner");
  ros::NodeHandle n;

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/model_description", model_urdf);

  n.getParam(ros::this_node::getName()+"/invert_spawn_positions", side_toggle);

  n.getParam(ros::this_node::getName()+"/x_pos_1", x_pos_1);
  n.getParam(ros::this_node::getName()+"/y_pos_1", y_pos_1);
  n.getParam(ros::this_node::getName()+"/z_pos_1", z_pos_1);
  n.getParam(ros::this_node::getName()+"/theta_1", theta_1);

  n.getParam(ros::this_node::getName()+"/x_pos_2", x_pos_1);
  n.getParam(ros::this_node::getName()+"/y_pos_2", y_pos_1);
  n.getParam(ros::this_node::getName()+"/z_pos_2", z_pos_1);
  n.getParam(ros::this_node::getName()+"/theta_2", theta_1);

  //Create all of the topic and service objects
  spawner_service = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_model");
  delete_service = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  ros::Subscriber move_base_results = n.subscribe("move_base/status", 10, state_callback);

  //Spawn in first object
  if(side_toggle)
  {
    spawn_object("dynamic_obstacle_1", x_pos_1, y_pos_1, z_pos_1, theta_1);
  }
  else
  {
    spawn_object("dynamic_obstacle_2", x_pos_2, y_pos_2, z_pos_2, theta_2);
  }

  //Maintain node and checks for susbcriber callback trigger
  ros::spin();

  return 0;
}
