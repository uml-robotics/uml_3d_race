#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <gazebo_msgs/ContactsState.h>
#include <string>
#include <sstream>
#include <list>
#include <cmath>

//SimLog is a custom message type defined in uml_3d_race/msg/SimLog.msg
#include <uml_3d_race/SimLog.h>

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
    //Publisher
    ros::Publisher pub;

    //Msg
    uml_3d_race::SimLog log;

    //Other variables
    uint collision_num;
    uint iteration_collision;
    double collision_x;
    double collision_y;
    double collision_thres;
    uint goals_reached;
    uint goals_aborted;
    bool navigating;
    bool publishing;

public:
    Referee(ros::NodeHandle n)
    {
        // Setup publisher
        pub = n.advertise<uml_3d_race::SimLog>("sim_log", 1000, false);

        collision_num = 0;
        iteration_collision = 0;
        collision_x = 0;
        collision_y = 0;
        collision_thres = 0.3;
        goals_reached = 0;
        goals_aborted = 0;
        navigating = false;
        publishing = false;

        log.collision.x = -100000;
        log.collision.y = -100000;

        log.iteration = 0;
    };

    void add_position(nav_msgs::Odometry odom)
    {
        if (navigating)
        {
            //save the position and distance
            log.robot_pos.x = odom.pose.pose.position.x;
            log.robot_pos.y = odom.pose.pose.position.y;
            log.robot_pos.theta = odom.pose.pose.orientation.z;
            log.dist_from_goal = distance(log.goal.x, log.goal.y, odom.pose.pose.position.x, odom.pose.pose.position.y);
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

            //At this point the collision registered is valid and needs to be logged

            //Intrement runs with collision counter if this is the first collision reported in the current iteration
            if (iteration_collision == 0)
            {
                collision_num++;
            }

            //Increment the num of collisions in the current iteration and save coords
            iteration_collision++;
            collision_x = collision.states[0].contact_positions[0].x;
            collision_y = collision.states[0].contact_positions[0].y;

            //Add collision to msg
            log.collision.x = collision_x;
            log.collision.y = collision_y;

            //Add msg log
            ROS_WARN("Collision detected at x:%.3f y:%.3f", log.collision.x, log.collision.y);
            std::stringstream str;
            str << "Collision detected at x:" << log.collision.x << " y:" << log.collision.y;
            log.event = str.str();
        }
    }

    void add_nav_result(bool goal_reached)
    {
        if (navigating)
        {
            //Change navigation state
            navigating = false;
            if (goal_reached)
            {
                //Add msg log
                ROS_INFO("The goal was reached");
                log.event = "Goal was reached";
            }
            else
            {
                //Add msg log
                ROS_WARN("The goal was not reached");
                log.event = "Goal was not reached";
            }

            //Output an iteration is ending if trips is even
            ROS_INFO("Ending iteration %d", log.iteration);
        }
    }

    void add_goal(geometry_msgs::Pose2D goal)
    {
        if (!navigating)
        {
            //Save goal pos
            log.goal.x = goal.x;
            log.goal.y = goal.y;

            //Change nav and publish state
            navigating = true;
            publishing = true;

            //Increment iterations
            log.iteration++;

            //Reset iteration collision counter
            iteration_collision = 0;

            //Add msg log
            std::stringstream str;
            str << "Goal registered at x:" << goal.x << " y:" << goal.y;
            log.event = str.str();
            ROS_INFO("Starting iteration %d", log.iteration);
            ROS_INFO("Goal registered at x:%.3f y:%.3f", goal.x, goal.y);
        }
    }

    void publish_log()
    {
        if (publishing)
        {
            log.header.stamp = ros::Time::now();
            pub.publish(log);

            //Reset collision coords if a collision was published
            if (log.collision.x != -100000 || log.collision.y != -100000)
            {
                log.collision.x = -100000;
                log.collision.y = -100000;
            }

            //Reset event string if an event was published
            if (log.event != "")
            {
                log.event.clear();
            }
        }
        if (!navigating)
        {
            publishing = false;
        }
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

void goal_callback(const geometry_msgs::Pose2D::ConstPtr &goal)
{
    logger->add_goal(*goal);
}

void state_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &state)
{
    if (!state->status_list.empty())
    {
        if (state->status_list.back().status == state->status_list.back().ABORTED || state->status_list.back().status == state->status_list.back().REJECTED)
        {
            logger->add_nav_result(false);
        }

        if (state->status_list.back().status == state->status_list.back().SUCCEEDED)
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

    ros::Rate rate(10);
    ros::AsyncSpinner spinner(5);

    // Setup subscribers
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odom_callback);
    ros::Subscriber collision_sub = n.subscribe("bumper_contact", 10, collision_callback);
    ros::Subscriber goal_sub = n.subscribe("/goal", 10, goal_callback);
    ros::Subscriber result = n.subscribe("move_base/status", 10, state_callback);

    logger = new Referee(n);

    spinner.start();

    while (ros::ok())
    {
        logger->publish_log();
        ros::spinOnce();
        rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}
