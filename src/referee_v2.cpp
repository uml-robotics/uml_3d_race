#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <gazebo_msgs/ContactsState.h>
#include <string>
#include <sstream>
#include <list>
#include <cmath>

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Simple distance function
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

class Referee
{
private:
    double goal_x;
    double goal_y;
    int collision_num;
    int iteration_collision;
    double collision_x;
    double collision_y;
    double collision_thres;
    int goals_reached;
    int goals_aborted;
    int current_iteration;
    bool navigating;

public:
    Referee()
    {
        goal_x = 0;
        goal_y = 0;
        collision_num = 0;
        iteration_collision = 0;
        collision_x = 0;
        collision_y = 0;
        collision_thres = 0.3;
        goals_reached = 0;
        goals_aborted = 0;
        current_iteration = 0;
        navigating = false;
    };

    void add_position(nav_msgs::Odometry odom)
    {
        if (navigating)
        {
            //Log the position
            ROS_INFO("Robot's position is x:%.3f y:%.3f and the distance to the goal is %.3f", odom.pose.pose.position.x, odom.pose.pose.position.y,
                     distance(goal_x, goal_y, odom.pose.pose.position.x, odom.pose.pose.position.y));
        }
    }

    void add_collision(gazebo_msgs::ContactsState collision)
    {
        if (navigating && !collision.states.empty())
        {
            //Filters out collisions that are right next to each other
            if (iteration_collision != 0 && std::abs(collision.states[0].contact_positions[0].x - collision_x < collision_thres) && std::abs(collision.states[0].contact_positions[0].y - collision_y < collision_thres))
            {
                return;
            }

            //Intrement runs with collision counter if this is the first collision reported in the current iteration
            if (iteration_collision == 0)
            {
                collision_num++;
            }

            //Increment the num of collisions in the current iteration and save coords
            iteration_collision++;
            collision_x = collision.states[0].contact_positions[0].x;
            collision_y = collision.states[0].contact_positions[0].y;

            //Log the collision
            ROS_INFO_STREAM(
                "******************************************************************************" << std::endl
                << std::endl
                << "Collision number " << iteration_collision << " detected at x:" << collision.states[0].contact_positions[0].x << " y:" << collision.states[0].contact_positions[0].y << std::endl
                << std::endl
                << "******************************************************************************"
            );
        }
    }

    void add_nav_result(bool goal_reached)
    {
        if (navigating)
        {
            //Determine if the robot reached the goal
            if (goal_reached)
            {
                //Increment goals reached counter
                goals_reached++;

                //Log
                ROS_INFO_STREAM(
                    "*****************************************************************************" << std::endl
                    << std::endl
                    << "Ending iteration " << current_iteration << ": The robot has reached the goal" << std::endl
                    << std::endl
                    << "*****************************************************************************"
                );
            }
            else
            {
                //Increment fail counter
                goals_aborted++;

                //Log
                ROS_INFO_STREAM(
                    "*****************************************************************************" << std::endl
                    << std::endl
                    << "Ending iteration " << current_iteration << ": The robot has failed to reach the goal" << std::endl
                    << std::endl
                    << "*****************************************************************************"
                );
            }

            //Change navigation state
            navigating = false;

            print_summary();
        }
    }

    void add_goal(uml_3d_race::Goal goal)
    {
        //Save goal coord for distance calculation
        goal_x = goal.x;
        goal_y = goal.y;

        navigating = true;

        current_iteration++;

        //Reset iteration collision counter
        iteration_collision = 0;

        //Log goal
        ROS_INFO_STREAM(
            "*****************************************************************************" << std::endl
            << std::endl
            << "Starting iteration " << current_iteration << ": Goal location is x:" << goal.x << " y:" << goal.y << std::endl
            << std::endl
            << "*****************************************************************************";
        );
    }

    void print_summary()
    {
        //Subtract the number of iterations that had collisions from the number of times the goal was reached
        goals_reached -= collision_num;

        ROS_WARN_STREAM(
            "*******************************************************************************" << std::endl
            << std::endl
            << "                         Simulation Summary" << std::endl
            << std::endl
            << "Number of times the goal was reached: " << goals_reached << std::endl
            << "Number of times the goal was reached, but the robot collided with an object: " << collision_num << std::endl
            << "Number of times the goal was not reached: " << goals_aborted << std::endl
            << std::endl
            << "*******************************************************************************"
        );
    }
};

Referee *logger;

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    logger->add_position(*odom);
}

void collision_callback(const gazebo_msgs::ContactsState::ConstPtr &collision)
{
    logger->add_collision(*collision);
}

void goal_callback(const uml_3d_race::Goal::ConstPtr &goal)
{
    logger->add_goal(*goal);
}

void state_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &state)
{
    if (!state->status_list.empty())
    {
        if (state->status_list.front().status == state->status_list.front().ABORTED || state->status_list.front().status == state->status_list.front().REJECTED)
        {
            logger->add_nav_result(false);
        }

        if (state->status_list.front().status == state->status_list.front().SUCCEEDED)
        {
            logger->add_nav_result(true);
        }
    }
}

int main(int argc, char **argv)
{
    // Setup ros node and NodeHandle
    ros::init(argc, argv, "referee");
    ros::NodeHandle n;

    // Setup subscribers and action clients
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odom_callback);
    ros::Subscriber collision_sub = n.subscribe("bumper_contact", 1, collision_callback);
    ros::Subscriber goal_sub = n.subscribe("/goal", 1, goal_callback);
    ros::Subscriber result = n.subscribe("move_base/status", 1, state_callback);

    logger = new Referee();

    //Loop and service the node
    ros::spin();
    
    return 0;
}
