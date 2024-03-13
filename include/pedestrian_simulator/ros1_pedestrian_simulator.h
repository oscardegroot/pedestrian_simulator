#ifndef ROS1_PEDESTRIAN_SIMULATOR_H
#define ROS1_PEDESTRIAN_SIMULATOR_H
#include <pedestrian_simulator/pedestrian_simulator.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
#include <mpc_planner_msgs/ObstacleArray.h>
#include <mpc_planner_msgs/Gaussian.h>

class Prediction;

class ROSPedestrianSimulator
{
public:
    ROSPedestrianSimulator();

    void ResetCallback(const std_msgs::Empty &msg);
    void VehicleVelocityCallback(const geometry_msgs::Twist &msg); /* For pretending that the vehicle is moving! */

    /** @brief Shift the origin to the origin of the reference path */
    void OriginCallback(const nav_msgs::Path &msg);
    void RobotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void SettingNCallback(const std_msgs::Int32 &msg);
    void SettingdtCallback(const std_msgs::Float32 &msg);
    void SettingHzCallback(const std_msgs::Float32 &msg);

    void Loop(const ros::TimerEvent &event);

private:
    ros::NodeHandle _nh;
    std::unique_ptr<PedestrianSimulator> _simulator;

    ros::Timer timer_;

    void InitializePublishersAndSubscribers();

    derived_object_msgs::ObjectArray PredictionsToObjectArray(const std::vector<Prediction> &predictions);
    mpc_planner_msgs::ObstacleArray PredictionsToObstacleArray(const std::vector<Prediction> &predictions);

    bool set_N_{false}, set_dt_{false}, set_hz_{false};

    ros::Publisher obstacle_pub_, obstacle_prediction_pub_, obstacle_trajectory_prediction_pub_;
    ros::Publisher ped_model_visuals_;
    ros::Publisher joystick_publisher_;

    ros::ServiceServer start_server;
    ros::Subscriber reset_sub_, vehicle_speed_sub_, path_origin_sub_;
    ros::Subscriber setting_N_sub_, setting_dt_sub_, setting_hz_sub_;
    ros::Subscriber robot_state_sub_;

    std::vector<ros::Publisher> optitrack_publishers_;
};

#endif // ROS1_PEDESTRIAN_SIMULATOR_H
