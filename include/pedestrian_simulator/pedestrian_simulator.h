#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <tf/tf.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>

#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include "lmpcc_tools/helpers.h"
#include "lmpcc_tools/ros_visuals.h"

#include "pedestrian_simulator/pedestrian.h"
#include "pedestrian_simulator/xml_reader.h"
#include "pedestrian_simulator/types.h"
#include "pedestrian_simulator/configuration.h"

class PedestrianSimulator
{

public:
    PedestrianSimulator();

public:
    Helpers::RandomGenerator random_generator_;

    void ResetCallback(const std_msgs::Empty &msg);
    void VehicleVelocityCallback(const geometry_msgs::Twist &msg); /* For pretending that the vehicle is moving! */

    /** @brief Shift the origin to the origin of the reference path */
    void OriginCallback(const nav_msgs::Path &msg);

    void Poll(const ros::TimerEvent &event);

    void Publish();
    void PublishTrajectoryPredictions();
    void PublishBinomialTrajectoryPredictions();

    void PublishDebugVisuals();
    void VisualizePedestrians();

private:
    ros::Timer timer_;
    ros::NodeHandle nh_;

    std::unique_ptr<XMLReader> xml_reader_;

    geometry_msgs::Pose origin_;

    ros::Publisher obstacle_pub_, obstacle_prediction_pub_, obstacle_trajectory_prediction_pub_;
    ros::Publisher ped_model_visuals_;
    std::vector<ros::Publisher> carla_position_pub_, carla_velocity_pub_;
    ros::Subscriber reset_sub_, vehicle_speed_sub_, path_origin_sub_;

    std::vector<std::unique_ptr<Pedestrian>> pedestrians_;

    std::unique_ptr<ROSMarkerPublisher> debug_visuals_;

    geometry_msgs::Pose vehicle_frame_; /* For pretending that the vehicle is moving! */

    std::vector<double> colors_ = {217, 83, 25, 0, 114, 189, 119, 172, 48, 126, 47, 142, 237, 177, 32, 77, 190, 238, 162, 19, 47, 256, 153, 256, 0, 103, 256};

    void Reset();
};

#endif // __PEDESTRIAN_SIMULATOR_H__