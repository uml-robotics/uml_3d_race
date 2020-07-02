#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "uml_3d_race/Goal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient *ac ;

void goalCallback( uml_3d_race::Goal goal){

    //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

       move_base_msgs::MoveBaseGoal cmd;

       cmd.target_pose.header.frame_id = "map";
       cmd.target_pose.header.stamp = ros::Time::now();

       cmd.target_pose.pose.position.x = goal.x;
       cmd.target_pose.pose.position.y = goal.y;
       cmd.target_pose.pose.orientation.w = 1.0;

       ROS_INFO("Sending goal");
       ac->sendGoal(cmd);

       ac->waitForResult();

}

int main (int argc, char **argv) {
    // Setup ros node and NodeHandle
    ros::init(argc,argv,"mover_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/goal",1000,goalCallback);
    ac = new MoveBaseClient("/move_base", true);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

return 0;

}
