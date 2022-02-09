#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>

#include <std_msgs/Empty.h>

#include "lmpcc_tools/helpers.h"
#include "lmpcc_tools/ros_visuals.h"
#include "pedestrian_simulator/xml_reader.h"

constexpr double DELTA_T = 0.05;
constexpr double VELOCITY = 1.3;
constexpr int HORIZON_N = 20;

struct Waypoint
{
    double x, y;

    Waypoint(){};

    Waypoint(double x, double y)
        : x(x), y(y){};

    double Distance(Waypoint &other)
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    double Angle(Waypoint &other)
    {
        return std::atan2(other.y - y, other.x - x);
    }
};

typedef std::vector<Waypoint> Path;

struct Pedestrian
{
    unsigned int id_;
    unsigned int current_path_id_, current_waypoint_id_;
    bool done_;

    Waypoint start_;
    Waypoint position_;

    geometry_msgs::Twist twist_;

    std::vector<Path> paths_;

    Pedestrian(const Waypoint &start)
    {
        start_ = start;
        Reset();
    }

    void Reset()
    {
        position_ = start_;
        current_waypoint_id_ = 0;
        done_ = false;
    }

    // Set the waypoint to the closest one
    bool FindClosestWaypoint()
    {
        double min_dist = 1e9;
        double direction = std::atan2(twist_.linear.y, twist_.linear.x);
        for (size_t waypoint_id = 0; waypoint_id < paths_[current_path_id_].size(); waypoint_id++)
        {
            double cur_dist = position_.Distance(paths_[current_path_id_][waypoint_id]);
            double angle = position_.Angle(paths_[current_path_id_][waypoint_id]);
            double angle_diff = std::abs(angle - direction);

            while (angle_diff > (2 * M_PI))
                angle_diff -= 2 * M_PI;

            // If we are already at the waypoint (i.e., direction doesn't matter) OR we are moving towards the waypoint
            if ((done_ || angle_diff < M_PI_2) && cur_dist < min_dist)
            {
                min_dist = cur_dist;
                current_waypoint_id_ = waypoint_id;
            }
        }
        if (min_dist == 1e9)
            return false;
        else
            return true;
    }

    void PickPath(Helpers::RandomGenerator &random_generator)
    {
        current_path_id_ = random_generator.Int(paths_.size());
    }

    bool HasReachedWaypoint()
    {
        return position_.Distance(GetCurrentWaypoint()) < 0.05;
    }

    Waypoint &GetCurrentWaypoint()
    {
        return paths_[current_path_id_][current_waypoint_id_];
    }

    void Update()
    {
        if (done_)
            return;

        // WAYPOINT BASED (I cannot compute the probability distribution for this)
        // Check if ped has reach the waypoint
        /*if (HasReachedWaypoint())
        {
            if (current_waypoint_id_ < paths_[current_path_id_].size() - 1)
                current_waypoint_id_++;
            else
            {
                // Stop moving
                done_ = true;
                twist_.linear.x = 0.;
                twist_.linear.y = 0.;
            }
        }
        // Move towards the waypoint
        Waypoint &cur_waypoint = GetCurrentWaypoint();
        twist_.linear.x = VELOCITY * std::cos(position_.Angle(cur_waypoint));
        twist_.linear.y = VELOCITY * std::sin(position_.Angle(cur_waypoint));*/

        position_.x += twist_.linear.x * DELTA_T;
        position_.y += twist_.linear.y * DELTA_T;
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

    void PredictPath(lmpcc_msgs::obstacle_gmm &msg)
    {
        for (size_t path_id = 0; path_id < paths_.size(); path_id++)
        {
            // Create a new pedestrian at the current position
            Pedestrian fake_ped = *this;
            fake_ped.current_path_id_ = path_id; // Explicitly set the path ID

            bool found_waypoint = fake_ped.FindClosestWaypoint(); // Set the current waypoint via a search
            if (!found_waypoint)
                continue;

            lmpcc_msgs::gaussian gaussian_msg;
            double major = 0.;
            double minor = 0.;
            for (int k = 0; k < HORIZON_N; k++)
            {
                fake_ped.Update(); // Update the fake ped
                geometry_msgs::PoseStamped cur_pose;
                cur_pose.pose.position.x = fake_ped.position_.x;
                cur_pose.pose.position.y = fake_ped.position_.y;
                gaussian_msg.mean.poses.push_back(cur_pose); // Add its position as mean of the gaussian

                major += std::pow(DELTA_T, 2.) * 1.0; // To be added in the real predictions?
                minor += std::pow(DELTA_T, 2.) * 1.0;

                gaussian_msg.major_semiaxis.push_back(major); // static for now
                gaussian_msg.minor_semiaxis.push_back(minor); // static for now
            }

            // We want to only add this prediction if the path is still reasonable
            msg.gaussians.push_back(gaussian_msg);
        }

        for (size_t g = 0; g < msg.gaussians.size(); g++)
            msg.probabilities.push_back(1. / ((double)msg.gaussians.size()));

        if (msg.gaussians.size() == 0)
            ROS_ERROR("No valid paths found!");
    }
};

class PedestrianSimulator
{

public:
    PedestrianSimulator();

public:
    void ResetCallback(const std_msgs::Empty &msg);

    void Poll(const ros::TimerEvent &event);

    void Publish();
    void PublishPredictions();
    Helpers::RandomGenerator random_generator_;

    void PublishDebugVisuals();

private:
    ros::Timer timer_;
    ros::NodeHandle nh_;

    ros::Publisher obstacle_pub_, obstacle_prediction_pub_;
    ros::Subscriber reset_sub_;

    std::vector<Pedestrian> pedestrians_;

    std::unique_ptr<ROSMarkerPublisher> debug_visuals_;

    void Reset();
};

#endif // __PEDESTRIAN_SIMULATOR_H__