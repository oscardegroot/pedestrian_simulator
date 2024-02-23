#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/xml_reader.h>
#include <pedestrian_simulator/pedsim_manager.h>

#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <derived_object_msgs/ObjectArray.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

class Pedestrian;
class PedestrianSimulator
{

public:
    PedestrianSimulator();

    // virtual ~PedestrianSimulator(){};

public:
    RosTools::RandomGenerator random_generator_;

    void Start();

    void ResetCallback(const std_msgs::Empty &msg);
    void VehicleVelocityCallback(const geometry_msgs::Twist &msg); /* For pretending that the vehicle is moving! */

    /** @brief Shift the origin to the origin of the reference path */
    void OriginCallback(const nav_msgs::Path &msg);
    void RobotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void SettingNCallback(const std_msgs::Int32 &msg);
    void SettingdtCallback(const std_msgs::Float32 &msg);
    void SettingHzCallback(const std_msgs::Float32 &msg);

    void Poll(const ros::TimerEvent &event);

    void Publish();

    void PublishOptitrackPedestrians(const derived_object_msgs::ObjectArray &ped_msg);

    /** @brief Trajectory predictions for deterministic models (e.g., social forces) */
    void PublishPredictions();
    void PublishGaussianPredictions();

    /** @brief Trajectory predictions for uncertain pedestrian model following a binomial distribution */
    void PublishBinomialPredictions();
    void PublishSocialPredictions();
    void PublishDebugVisuals();
    void VisualizeRobot();
    void VisualizePedestrians();
    void VisualizeStaticObstacles();

private:
    ros::Timer timer_;
    ros::NodeHandle nh_;

    std::unique_ptr<XMLReader> xml_reader_;
    std::unique_ptr<PedsimManager> pedsim_manager_, pedsim_prediction_manager_;

    geometry_msgs::Pose origin_;

    RobotState robot_state_;

    ros::Publisher obstacle_pub_, obstacle_prediction_pub_, obstacle_trajectory_prediction_pub_;
    ros::Publisher ped_model_visuals_;
    ros::Publisher joystick_publisher_;

    ros::ServiceServer start_server;

    std::vector<ros::Publisher> optitrack_publishers_;
    std::vector<ros::Publisher> carla_position_pub_, carla_velocity_pub_;

    ros::Subscriber reset_sub_, vehicle_speed_sub_, path_origin_sub_;
    ros::Subscriber setting_N_sub_, setting_dt_sub_, setting_hz_sub_;
    ros::Subscriber robot_state_sub_;

    std::vector<std::unique_ptr<Pedestrian>> pedestrians_;

    std::unique_ptr<RosTools::ROSMarkerPublisher> debug_visuals_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> static_obstacle_visuals_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> robot_visual_;

    geometry_msgs::Pose vehicle_frame_; /* For pretending that the vehicle is moving! */

    std::vector<double> colors_ = {217, 83, 25, 0, 114, 189, 119, 172, 48, 126, 47, 142, 237, 177, 32, 77, 190, 238, 162, 19, 47, 256, 153, 256, 0, 103, 256};
    void Reset();
};

#endif // __PEDESTRIAN_SIMULATOR_H__