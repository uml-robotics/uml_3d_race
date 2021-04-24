/*  The reset_robot node resets the robot in Gazebo to allow the robot to execute another navigation test without killing Gazebo.
    To reset the robot, the robot is first stopped before moving its position.  Then, the Gazebo model for the robot is set to
    its initial position from the pose specified in the spawn topic.  Also, the AMCL node's position is reset to the pose found
    in the spawn topic and the costmaps in the robot are reset.
*/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>

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
  
  //Get the name of the robot in gazebo
  n.getParam(ros::this_node::getName()+"/model_name", model_name);

  //Setup all of the publishers, subscribers, and services needed
  ros::Subscriber sub = n.subscribe("spawn", 1, spawn_callback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(model_name+"/cmd_vel", 1000);
  ros::Publisher odom_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  ros::Publisher amcl_pub = n.advertise<PoseWithCovariance>("/amcl/initialpose", 1000);
  ros::ServiceClient costmap_service = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

  //wait for services
  costmap_service.waitForExistence();

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

      //Construct empty service msg for resetting costmaps
      std_srvs::Empty service_msg;

      ROS_INFO("Stopping robot...");
      vel_pub.publish(stopped);
      ROS_INFO("Resetting robot position...");
      odom_pub.publish(spawn);
      amcl_pub.publish(pose);
      if (!costmap_service.call(service_msg))
      {
          ROS_WARN("Failed to clear costmaps");
      }
      
      //Kill the node after resetting the robot
      ros::Duration(0.5).sleep();
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
