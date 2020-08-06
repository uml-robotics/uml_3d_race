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

const float OFFICIAL_GOAL_TOL = 0.9;

float goal_x;
float goal_y;
float goal_tolerance;
int state = -1;
ros::Time start, finish;
float vel;
geometry_msgs::Point last_pos;

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
    double collision_num;
    double collision_x;
    double collision_y;
    double collision_thres;
    bool reached_goal;

public:
    Referee()
    {
        goal_x = 0;
        goal_y = 0;
        collision_num = 0;
        collision_x = 0;
        collision_y = 0;
        collision_thres = 0.2;
        reached_goal = false;
    };

    void add_position(nav_msgs::Odometry odom)
    {
        //Log the position
        ROS_INFO("Robot's position is x:%.2f y:%.2f and the distance to the goal is %.2f", odom.pose.pose.position.x, odom.pose.pose.position.y,
                 distance(goal_x, goal_y, odom.pose.pose.position.x, odom.pose.pose.position.y));
    }

    void add_collision(gazebo_msgs::ContactsState collision)
    {
        if (!collision.states.empty())
        {
            //Filter out collisions that are right next to each other
            if (collision_num != 0 && std::abs(collision.states[0].contact_positions[0].x - collision_x < collision_thres) && std::abs(collision.states[0].contact_positions[0].y - collision_y < collision_thres))
            {
                return;
            }

            //Increment collision_num and save coords
            collision_num++;
            collision_x = collision.states[0].contact_positions[0].x;
            collision_y = collision.states[0].contact_positions[0].y;

            //Log the collision
            ROS_INFO("Collision detected at x:%.2f y:%.2f", collision.states[0].contact_positions[0].x, collision.states[0].contact_positions[0].y);
        }
    }

    void add_nav_result(bool goal_reached)
    {
        //Save result
        reached_goal = goal_reached;

        //Log the result
        if (reached_goal)
        {
            ROS_INFO("Robot has reached the goal");
        }
        else
        {
            ROS_INFO("Robot failed to reach the goal");
        }

        //Stop the logger
        ros::shutdown();
    }

    void add_goal(uml_3d_race::Goal goal)
    {
        //Save goal coord for distance calculation
        goal_x = goal.x;
        goal_y = goal.y;

        //Log goal
        ROS_INFO("Goal location is x:%.2f y:%.2f", goal.x, goal.y);
    }
};

Referee *logger;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    logger->add_position(*odom);
}

void collision_callback(const gazebo_msgs::ContactsState::ConstPtr& collision)
{
    logger->add_collision(*collision);
}

void goal_callback(const uml_3d_race::Goal::ConstPtr& goal)
{
    logger->add_goal(*goal);
}

void state_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& state)
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

    ros::spin();

    return 0;
}
