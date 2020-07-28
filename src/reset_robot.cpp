#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef geometry_msgs::PoseWithCovarianceStamped PoseWithCovariance;

bool msg_received = false;
PoseWithCovariance poseMsg;

void spawn_callback(const geometry_msgs::PoseWithCovarianceStamped& spawn){
  poseMsg = spawn;
  msg_received = true;
}

int main(int argc, char **argv){
  // Setup ros node and NodeHandle
  ros::init(argc, argv, "robot_reset_node");
  ros::NodeHandle n;

  //Set defaults
  std::string model_name = "pioneer";
  std::string topic = "pioneer/spawn";
  
  n.getParam(ros::this_node::getName()+"/topic",topic);
  n.getParam(ros::this_node::getName()+"/model_name",model_name);

  ros::Subscriber sub = n.subscribe(topic, 1, spawn_callback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(model_name+"/cmd_vel", 1000);
  ros::Publisher odom_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  ros::Publisher amcl_pub = n.advertise<PoseWithCovariance>("/initialpose", 1000);

  // Node waits to receive a message from /spawn topics, then publishes the message
  // to /cmd_vel and /odom to reset the state of the robot to where it originally spawned.
  // Node then shuts itself down.
  ros::Rate loop_rate(1);
  while (ros::ok()){
    if(msg_received){
      // Twist and Pose defaults with zeroes, set what's relevant.
      geometry_msgs::Twist stopped;
      PoseWithCovariance pose;
      pose.header.frame_id = pose.header.frame_id;
      pose.header.stamp = ros::Time::now();
      pose.pose.pose.position.x = poseMsg.pose.pose.position.x;
      pose.pose.pose.position.y = poseMsg.pose.pose.position.y;
      pose.pose.pose.orientation.z = poseMsg.pose.pose.orientation.z;
      pose.pose.pose.orientation.w = poseMsg.pose.pose.orientation.w;

      //Construct respawn ModelState
      gazebo_msgs::ModelState spawn;
      spawn.model_name = model_name;
      spawn.pose = pose.pose.pose;
      spawn.twist = stopped;
      spawn.reference_frame = "world";

      ROS_INFO("Stopping robot...");
      vel_pub.publish(stopped);
      ROS_INFO("Resetting robot position...");
      odom_pub.publish(spawn);
      amcl_pub.publish(pose);
      
      
      ros::Duration(0.5).sleep();
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
