/**
 * @file collision_checker_node.cpp
 * @author Oscar de Groot
 * @brief Read obstacle and robot positions and check for collisions (published)
 * @version 0.2
 * @date 2024-10-16
 */

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>
#include <ros_tools/convertions.h>

rclcpp::Time state_received_time, obstacles_received_time;

Eigen::Vector2d robot_pos;
std::vector<Eigen::Vector2d> obstacle_pos;
double radius_obstacle, radius_robot, robot_offset;

rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub;
rclcpp::Time reset_time;

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr collision_pub;

double prev_intrusion = -1.;

void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    reset_time = rclcpp::Clock().now();
}

void Callback(nav_msgs::msg::Odometry::SharedPtr robot_msg, mpc_planner_msgs::msg::ObstacleArray::SharedPtr obstacle_msg)
{
    // if (reset_time + rclcpp::Duration(1.0) > rclcpp::Clock().now()) // When we received a reset, stop checking for collisions
    // return;

    state_received_time = robot_msg->header.stamp;

    // Obstacles
    obstacles_received_time = obstacle_msg->header.stamp;

    obstacle_pos.clear();
    for (auto &agent_state : obstacle_msg->obstacles)
        obstacle_pos.emplace_back(agent_state.pose.position.x, agent_state.pose.position.y);

    // std::cout << "checking collisions...\n"
    //           << "State received " << (rclcpp::Clock().now() - state_received_time).nanoseconds() / 1.0e6 << "ms ago\n "
    //           << "Obstacles received " << (rclcpp::Clock().now() - obstacles_received_time).nanoseconds() / 1.0e6 << "ms ago" << std::endl;

    int collisions = 0;
    double max_intrusion = 0.;
    double robot_angle = RosTools::quaternionToAngle(robot_msg->pose.pose.orientation);

    for (size_t i = 0; i < 3; i++)
    {
        robot_pos = Eigen::Vector2d(robot_msg->pose.pose.position.x, robot_msg->pose.pose.position.y);
        if (i == 0)
            robot_pos -= robot_offset * Eigen::Vector2d(std::cos(robot_angle), std::sin(robot_angle));

        if (i == 2)
            robot_pos += robot_offset * Eigen::Vector2d(std::cos(robot_angle), std::sin(robot_angle));

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
    }

    if (max_intrusion > 0. && prev_intrusion == 0.)
        RCLCPP_WARN(rclcpp::get_logger("collision_checker"), "Collision Detected. Intrusion: %fm", max_intrusion);

    if (max_intrusion != prev_intrusion)
    {
        auto intrusion_msg = std_msgs::msg::Float64();
        intrusion_msg.data = max_intrusion;
        collision_pub->publish(intrusion_msg);
        prev_intrusion = max_intrusion;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("collision_checker_node");

    collision_pub = node->create_publisher<std_msgs::msg::Float64>("pedestrian_simulator/collision_detected", 1);

    // Initialize subscribers and publishers
    reset_sub = node->create_subscription<std_msgs::msg::Empty>("/lmpcc/reset_environment", 1, ResetCallback);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, mpc_planner_msgs::msg::ObstacleArray> MySyncPolicy;
    // message_filters::Subscriber<example_interfaces::msg::UInt32> sub(node, "my_topic", 1);

    // std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Temperature, sensor_msgs::msg::FluidPressure>>>(
    // message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Temperature,
    // sensor_msgs::msg::FluidPressure>(queue_size),
    // temp_sub, fluid_sub);
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default; // Or define your own QoS settings

    message_filters::Subscriber<nav_msgs::msg::Odometry> robot_sub(node.get(), "/localization/kinematic_state", custom_qos);
    message_filters::Subscriber<mpc_planner_msgs::msg::ObstacleArray> obstacles_sub(node.get(), "/pedestrian_simulator/trajectory_predictions", custom_qos);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), robot_sub, obstacles_sub);
    sync.registerCallback(&Callback);

    // Get parameters (radii)
    // node->get_parameter_or("obstacles/radius", radius_obstacle, 0.4);
    // node->get_parameter_or("robot/width", radius_robot, 0.325);
    // radius_robot /= 2.; // Radius is half the width

    radius_obstacle = 0.6 - 0.1; // 0.1 = margin
    radius_robot = 1.125;
    robot_offset = 1.275;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
