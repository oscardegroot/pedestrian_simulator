#ifndef __PEDESTRIAN_H__
#define __PEDESTRIAN_H__

#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>

#include "pedestrian_simulator/types.h"
#include "pedestrian_simulator/configuration.h"

struct Waypoint;
enum class PedState
{
    STRAIGHT = 0,
    CROSS
};

class Pedestrian
{
public:
    Pedestrian(const Waypoint &start)
    {
        start_ = start;
        Reset();
    }

public:
    virtual void Reset()
    {
        position_ = start_;
    }

    virtual void Update() = 0;
    virtual void PredictPath(lmpcc_msgs::obstacle_gmm &msg) = 0;

    unsigned int id_;

    Waypoint start_;
    Waypoint position_;

    geometry_msgs::Twist twist_;
};

class GaussianPedestrian : public Pedestrian
{
public:
    double angle;
    Eigen::Vector2d B;

    GaussianPedestrian(const Waypoint &start, const Waypoint &end)
        : Pedestrian(start)
    {
        Reset();
        // Set the dynamics
        angle = std::atan2(end.y - start.y, end.x - start.x);
        B = Eigen::Vector2d(
            std::cos(angle),
            std::sin(angle));
    }

    virtual void Update() override
    {
        Helpers::RandomGenerator random_generator;

        Eigen::Vector2d process_noise_realization = random_generator.BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                                       CONFIG.process_noise_[0],
                                                                                       CONFIG.process_noise_[1],
                                                                                       angle);

        twist_.linear.x = B(0) * CONFIG.ped_velocity_;
        twist_.linear.y = B(1) * CONFIG.ped_velocity_;

        // Major / minor -> Cov = [major^2, 0; 0 minor^2]
        position_.x += twist_.linear.x * DELTA_T + process_noise_realization(0) * DELTA_T;
        position_.y += twist_.linear.y * DELTA_T + process_noise_realization(1) * DELTA_T;
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

    virtual void PredictPath(lmpcc_msgs::obstacle_gmm &msg) override
    {
        lmpcc_msgs::gaussian gaussian_msg;
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = position_.x;
        new_pose.pose.position.y = position_.y;

        double major = 0.;
        double minor = 0.;

        for (int k = 0; k < HORIZON_N; k++)
        {
            new_pose.pose.position.x += twist_.linear.x * DELTA_T_PREDICT;
            new_pose.pose.position.y += twist_.linear.y * DELTA_T_PREDICT;
            gaussian_msg.mean.poses.push_back(new_pose);

            major += CONFIG.process_noise_[0] * DELTA_T_PREDICT;
            minor += CONFIG.process_noise_[1] * DELTA_T_PREDICT;

            gaussian_msg.major_semiaxis.push_back(major);
            gaussian_msg.minor_semiaxis.push_back(minor);
        }
        msg.gaussians.push_back(gaussian_msg);
        msg.probabilities.push_back(1.0);
    }

public:
    virtual void Reset()
    {
        Pedestrian::Reset();
    }
};

class BinomialPedestrian : public Pedestrian
{
public:
    BinomialPedestrian(const Waypoint &start)
        : Pedestrian(start)
    {
        Reset();
    }

    virtual void Update() override
    {
        Helpers::RandomGenerator random_generator;

        // BINOMIAL
        switch (state_)
        {
        case PedState::STRAIGHT:

            twist_.linear.x = CONFIG.ped_velocity_ * direction_;
            twist_.linear.y = 0.;

            // Transition
            if (random_generator.Double() > 0.999995)
                state_ = PedState::CROSS;

            break;
        case PedState::CROSS:

            twist_.linear.x = 0.;
            twist_.linear.y = CONFIG.ped_velocity_ * direction_;

            break;
        }

        position_.x += twist_.linear.x * DELTA_T;
        position_.y += twist_.linear.y * DELTA_T;
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

    virtual void PredictPath(lmpcc_msgs::obstacle_gmm &msg) override
    {
    }

public:
    virtual void Reset()
    {
        Pedestrian::Reset();
        state_ = PedState::STRAIGHT;

        // Temporary
        if (start_.x > 6)
            direction_ = -1;
        else
            direction_ = 1;
    }

    PedState state_;
    int direction_;
};

class WaypointPedestrian : public Pedestrian
{
public:
    WaypointPedestrian(const Waypoint &start)
        : Pedestrian(start)
    {
    }

public:
    unsigned int current_path_id_, current_waypoint_id_;
    std::vector<Path> paths_;

    void Reset() override
    {
        Pedestrian::Reset();

        current_waypoint_id_ = 0;
        done_ = false;

        PickPath();
    }

    void Update() override
    {
        if (done_)
            return;

        // WAYPOINT BASED (I cannot compute the probability distribution for this)
        // Check if ped has reach the waypoint
        if (HasReachedWaypoint())
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
        twist_.linear.x = CONFIG.ped_velocity_ * std::cos(position_.Angle(cur_waypoint));
        twist_.linear.y = CONFIG.ped_velocity_ * std::sin(position_.Angle(cur_waypoint));

        position_.x += twist_.linear.x * DELTA_T;
        position_.y += twist_.linear.y * DELTA_T;
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

    void PredictPath(lmpcc_msgs::obstacle_gmm &msg) override
    {
        for (size_t path_id = 0; path_id < paths_.size(); path_id++)
        {
            // Create a new pedestrian at the current position
            WaypointPedestrian fake_ped = *this;
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

    void PickPath()
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

protected:
    bool done_;

    Helpers::RandomGenerator random_generator;
};

#endif // __PEDESTRIAN_H__