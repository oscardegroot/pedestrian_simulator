#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/xml_reader.h>
#include <pedestrian_simulator/pedsim_manager.h>
#include <pedestrian_simulator/autoware_interface.h>

#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

#include <lmpcc_msgs/msg/obstacle_array.hpp>
#include <lmpcc_msgs/msg/obstacle_gmm.hpp>
#include <lmpcc_msgs/msg/gaussian.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <derived_object_msgs/msg/object_array.hpp>

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace pedestrian_simulator
{
    class Pedestrian;

    class PedestrianSimulator : public rclcpp::Node
    {

    public:
        PedestrianSimulator(const rclcpp::NodeOptions &options);

        // virtual ~PedestrianSimulator(){};

    public:
        RosTools::RandomGenerator random_generator_;

        void ResetCallback(const std_msgs::msg::Empty &msg);
        void VehicleVelocityCallback(const geometry_msgs::msg::Twist &msg); /* For pretending that the vehicle is moving! */

        /** @brief Shift the origin to the origin of the reference path */
        void OriginCallback(const nav_msgs::msg::Path &msg);
        void RobotStateCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);

        void SettingNCallback(const std_msgs::msg::Float64 &msg);
        void SettingdtCallback(const std_msgs::msg::Float64 &msg);
        void SettingHzCallback(const std_msgs::msg::Float64 &msg);

        void Poll();

        void Publish();

        void PublishOptitrackPedestrians(const derived_object_msgs::msg::ObjectArray &ped_msg);

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
        rclcpp::TimerBase::SharedPtr timer_;
        int times_;
        // rclcpp::Node::SharedPtr node_;

        std::unique_ptr<XMLReader> xml_reader_;
        std::unique_ptr<PedsimManager> pedsim_manager_, pedsim_prediction_manager_;

        geometry_msgs::msg::Pose origin_;

        RobotState robot_state_;

        std::unique_ptr<AutowareInterface> autoware_interface_;

        // rclcpp::Publisher<derived_object_msgs::msg::ObjectArray>::SharedPtr obstacle_pub_;
        rclcpp::Publisher<lmpcc_msgs::msg::ObstacleArray>::SharedPtr obstacle_prediction_pub_;
        rclcpp::Publisher<lmpcc_msgs::msg::ObstacleArray>::SharedPtr obstacle_trajectory_prediction_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ped_model_visuals_;
        // rclcpp::Publisher<>::SharedPtr obstacle_prediction_pub_;

        // ros::Publisher joystick_publisher_;
        // std::vector<ros::Publisher> optitrack_publishers_;
        // ros::Subscription<>::SharedPtr vehicle_speed_sub_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_origin_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setting_N_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setting_dt_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setting_hz_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_sub_;

        std::vector<std::unique_ptr<Pedestrian>> pedestrians_;

        std::unique_ptr<RosTools::ROSMarkerPublisher> debug_visuals_;
        std::unique_ptr<RosTools::ROSMarkerPublisher> static_obstacle_visuals_;
        std::unique_ptr<RosTools::ROSMarkerPublisher> robot_visual_;

        geometry_msgs::msg::Pose vehicle_frame_; /* For pretending that the vehicle is moving! */

        std::vector<double> colors_ = {217, 83, 25, 0, 114, 189, 119, 172, 48, 126, 47, 142, 237, 177, 32, 77, 190, 238, 162, 19, 47, 256, 153, 256, 0, 103, 256};
        void Reset();
    };
};

#endif // __PEDESTRIAN_SIMULATOR_H__