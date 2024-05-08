/**
 * @file collision_checker_node.cpp
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Read obstacle and robot positions and check for collisions (published)
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <Eigen/Dense>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

#include <mpc_planner_msgs/ObstacleArray.h>

ros::Time state_received_time, obstacles_received_time;

Eigen::Vector2d robot_pos;
std::vector<Eigen::Vector2d> obstacle_pos;
double radius_obstacle, radius_robot;

ros::Subscriber reset_sub;
ros::Time reset_time;

ros::Publisher collision_pub;

double prev_intrusion = -1.;

void ResetCallback(const std_msgs::Empty &msg)
{
    reset_time = ros::Time::now();
}

void Callback(const geometry_msgs::PoseStampedConstPtr &robot_msg, const mpc_planner_msgs::ObstacleArray::ConstPtr &obstacle_msg)
{
    if (reset_time + ros::Duration(1.) > ros::Time::now()) // When we received a reset, stop checking for collisions
        return;

    state_received_time = robot_msg->header.stamp;
    robot_pos = Eigen::Vector2d(robot_msg->pose.position.x, robot_msg->pose.position.y);

    // Obstacles
    obstacles_received_time = obstacle_msg->header.stamp;

    obstacle_pos.clear();
    for (auto &agent_state : obstacle_msg->obstacles)
        obstacle_pos.emplace_back(agent_state.pose.position.x, agent_state.pose.position.y);

    // std::cout << "checking collisions...\n"
    //           << "State received " << (ros::Time::now() - state_received_time).nsec / 1.0e6 << "ms ago\n "
    //           << "Obstacles received " << (ros::Time::now() - obstacles_received_time).nsec / 1.0e6 << "ms ago" << std::endl;

    int collisions = 0;
    double max_intrusion = 0.;
    for (auto &obstacle : obstacle_pos)
    {
        double distance = (robot_pos - obstacle).norm() - (radius_robot + radius_obstacle);
        if (distance < 0)
        {
            double intrusion = -distance;

            collisions++;
            max_intrusion = std::max(intrusion, max_intrusion);
        }
    }

    if (max_intrusion > 0. && prev_intrusion == 0.)
        ROS_WARN_STREAM("Collision Detected. Intrusion: " << max_intrusion << "m");

    if (max_intrusion != prev_intrusion)
    {
        std_msgs::Float64 intrusion_msg;
        intrusion_msg.data = max_intrusion;
        collision_pub.publish(intrusion_msg);
        prev_intrusion = max_intrusion;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;
    collision_pub = nh.advertise<std_msgs::Float64>("pedestrian_simulator/collision_detected", 1);

    // Initialize subscribers and publishers
    reset_sub = nh.subscribe("/lmpcc/reset_environment", 1, &ResetCallback);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mpc_planner_msgs::ObstacleArray> MySyncPolicy;

    message_filters::Subscriber<geometry_msgs::PoseStamped> robot_sub(nh, "robot_state", 3);
    message_filters::Subscriber<mpc_planner_msgs::ObstacleArray> obstacles_sub(nh, "/pedestrian_simulator/trajectory_predictions", 3);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), robot_sub, obstacles_sub);
    sync.registerCallback(boost::bind(&Callback, _1, _2));

    // Get parameters (radii)
    // nh.getParam("obstacles/radius", radius_obstacle, 0.4);
    // nh.getParam("robot/width", radius_robot, 0.325);
    // radius_robot /= 2.; // Radius is half the width

    radius_obstacle = 0.3;
    radius_robot = 0.325;

    ros::spin();

    return 0;
}
