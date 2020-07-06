#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;

void goalCallback(uml_3d_race::Goal goalMsg) {
    move_base_msgs::MoveBaseGoal goal;

    //translate Goal Message to a MoveBaseGoal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goalMsg.x;
    goal.target_pose.pose.position.y = goalMsg.y;
    goal.target_pose.pose.orientation.w = 1.0;

    //Send the goal to the navigation action client
    ac->sendGoal(goal);

    //Wait for the robot to finish moving
    ac->waitForResult();

    return;
}

int main(int argc, char **argv) {
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "mover_node");
    ros::NodeHandle n;

    ac = new MoveBaseClient("move_base", true);

    //wait for the action server to come up
    while (!ac->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Subscribe to the goal topic
    ros::Subscriber sub = n.subscribe("goal/", 1, goalCallback);

    ros::spin();

    return 0;
}
