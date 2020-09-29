#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::ServiceClient costmap_service;
ros::ServiceClient goal_service;

int goals_reached;
int iterations;

void goalCallback(uml_3d_race::Goal goal_msg)
{
    move_base_msgs::MoveBaseGoal goal;

    //translate Goal Message to a MoveBaseGoal
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_msg.x;
    goal.target_pose.pose.position.y = goal_msg.y;
    goal.target_pose.pose.orientation.w = 1.0;

    //Send the goal to the navigation action client
    ac->sendGoal(goal);

    //Wait for the robot to finish moving
    ac->waitForResult();

    //Clear costmaps once finished navigating
    std_srvs::Empty service_msg;
    if (!costmap_service.call(service_msg))
    {
        ROS_WARN("Failed to clear costmaps");
    }

    //Increase goals_reached counter
    goals_reached++;

    //Requests another goal if the number of trips has not been met, otherwise shut down the node
    if (goals_reached / 2 < iterations)
    {
        //Wait a little bit before requesting the next goal
        ros::Duration(2).sleep();

        //Get the next goal
        if (!goal_service.call(service_msg))
        {
            ROS_ERROR("Failed to request for a goal");
        }
    }
    else
    {
        //All iterations finished, node no longer needed
        ros::shutdown();
        return;
    }
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "mover_node");
    ros::NodeHandle n;

    //Subscribe to the goal topic
    ros::Subscriber sub = n.subscribe("goal", 10, goalCallback);
    
    ac = new MoveBaseClient("move_base", true);
    costmap_service = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    goal_service = n.serviceClient<std_srvs::Empty>("get_new_goal");

    //Set defaults
    goals_reached = 0;
    iterations = 0;

    //Get number of iterations
    n.getParam(ros::this_node::getName() + "/iterations", iterations);

    //wait for the action server to come up
    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //wait for services
    costmap_service.waitForExistence();
    goal_service.waitForExistence();

    //Call the goal service to recieve the first goal
    std_srvs::Empty service_msg;
    if (!goal_service.call(service_msg))
    {
        ROS_ERROR("Failed to request for a goal");
    }

    //Keeps the node running and performs necessary updates
    ros::spin();

    return 0;
}
