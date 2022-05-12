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
    virtual void MoveFrame(const Eigen::Vector2d &speed)
    {
        position_.x -= speed(0) * CONFIG.delta_t_;
        position_.y -= speed(1) * CONFIG.delta_t_;
    };

    unsigned int id_;

    Waypoint start_;
    Waypoint position_;

    geometry_msgs::Twist twist_;
    geometry_msgs::Twist noisy_twist_;
};

class GaussianPedestrian : public Pedestrian
{
public:
    double angle;
    Eigen::Vector2d B;
    std::unique_ptr<Helpers::RandomGenerator> random_generator_;
    int cur_seed_, seed_mp_;

    GaussianPedestrian(const Waypoint &start, const Waypoint &end, int seed_mp)
        : Pedestrian(start), seed_mp_(seed_mp)
    {
        cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_; // At initialization: define the start seed of this ped

        Reset();

        // Set the dynamics
        angle = std::atan2(end.y - start.y, end.x - start.x);
        B = Eigen::Vector2d(
            std::cos(angle),
            std::sin(angle));
    }

    virtual void Update() override
    {

        Eigen::Vector2d process_noise_realization = random_generator_->BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                                         CONFIG.process_noise_[0],
                                                                                         CONFIG.process_noise_[1],
                                                                                         angle);

        twist_.linear.x = B(0) * CONFIG.ped_velocity_;
        twist_.linear.y = B(1) * CONFIG.ped_velocity_;
        noisy_twist_ = twist_;

        // Major / minor -> Cov = [major^2, 0; 0 minor^2]
        if (!CONFIG.static_) // If static, do not update the position
        {
            noisy_twist_.linear.x += process_noise_realization(0);
            noisy_twist_.linear.y += process_noise_realization(1);
            position_.x += noisy_twist_.linear.x * CONFIG.delta_t_;
            position_.y += noisy_twist_.linear.y * CONFIG.delta_t_;
            // position_.x += twist_.linear.x * CONFIG.delta_t_ + process_noise_realization(0) * CONFIG.delta_t_;
            // position_.y += twist_.linear.y * CONFIG.delta_t_ + process_noise_realization(1) * CONFIG.delta_t_;
        }
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

public:
    virtual void Reset()
    {
        cur_seed_++; // We increase the seed after every simulation, to keep the behavior the same during each simulation
        random_generator_.reset(new Helpers::RandomGenerator(cur_seed_));

        Pedestrian::Reset();
    }
};

class BinomialPedestrian : public Pedestrian
{
public:
    Eigen::Vector2d B_straight;
    Eigen::Vector2d B_cross;
    PedState state;
    double p;
    int counter;
    std::unique_ptr<Helpers::RandomGenerator> random_generator_;
    int seed_mp_, cur_seed_;

    BinomialPedestrian(const Waypoint &start, int seed_mp)
        : Pedestrian(start), seed_mp_(seed_mp)
    {
        cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_;

        p = CONFIG.p_binomial_;
        Reset();
    }

    virtual void Update() override
    {

        // BINOMIAL
        switch (state)
        {
        case PedState::STRAIGHT:

            twist_.linear.x = B_straight(0) * CONFIG.ped_velocity_ * direction_;
            twist_.linear.y = B_straight(1) * CONFIG.ped_velocity_ * direction_;

            // Transition
            if ((counter % 4 == 0) && (random_generator_->Double() <= p)) // Do this only once every 4 times
                state = PedState::CROSS;

            counter = (counter + 1) % 4;

            break;
        case PedState::CROSS:

            twist_.linear.x = B_cross(0) * CONFIG.ped_velocity_ * direction_;
            twist_.linear.y = B_cross(1) * CONFIG.ped_velocity_ * direction_;

            break;
        }

        Eigen::Vector2d process_noise_realization = random_generator_->BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                                         CONFIG.process_noise_[0],
                                                                                         CONFIG.process_noise_[1],
                                                                                         0.);

        // Update the position using the velocity and Gaussian process noise
        position_.x += twist_.linear.x * CONFIG.delta_t_ + process_noise_realization(0) * CONFIG.delta_t_;
        position_.y += twist_.linear.y * CONFIG.delta_t_ + process_noise_realization(1) * CONFIG.delta_t_;
    }

public:
    virtual void Reset()
    {
        cur_seed_++;
        random_generator_.reset(new Helpers::RandomGenerator(cur_seed_));

        Pedestrian::Reset();

        // Set the dynamics
        B_straight = Eigen::Vector2d(1., 0.);
        B_cross = Eigen::Vector2d(1.0 / std::sqrt(2), 1.0 / std::sqrt(2));
        state = PedState::STRAIGHT;
        counter = 0;

        direction_ = start_.x < 6. ? 1. : -1.;
    }

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

        position_.x += twist_.linear.x * CONFIG.delta_t_;
        position_.y += twist_.linear.y * CONFIG.delta_t_;
        // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
    }

    // void PredictPath(lmpcc_msgs::obstacle_gmm &msg) override
    // {
    //     for (size_t path_id = 0; path_id < paths_.size(); path_id++)
    //     {
    //         // Create a new pedestrian at the current position
    //         WaypointPedestrian fake_ped = *this;
    //         fake_ped.current_path_id_ = path_id; // Explicitly set the path ID

    //         bool found_waypoint = fake_ped.FindClosestWaypoint(); // Set the current waypoint via a search
    //         if (!found_waypoint)
    //             continue;

    //         lmpcc_msgs::gaussian gaussian_msg;
    //         double major = 0.;
    //         double minor = 0.;
    //         for (int k = 0; k < HORIZON_N; k++)
    //         {
    //             fake_ped.Update(); // Update the fake ped
    //             geometry_msgs::PoseStamped cur_pose;
    //             cur_pose.pose.position.x = fake_ped.position_.x;
    //             cur_pose.pose.position.y = fake_ped.position_.y;
    //             gaussian_msg.mean.poses.push_back(cur_pose); // Add its position as mean of the gaussian

    //             major += std::pow(CONFIG.delta_t_, 2.) * 1.0; // To be added in the real predictions?
    //             minor += std::pow(CONFIG.delta_t_, 2.) * 1.0;

    //             gaussian_msg.major_semiaxis.push_back(major); // static for now
    //             gaussian_msg.minor_semiaxis.push_back(minor); // static for now
    //         }

    //         // We want to only add this prediction if the path is still reasonable
    //         msg.gaussians.push_back(gaussian_msg);
    //     }

    //     for (size_t g = 0; g < msg.gaussians.size(); g++)
    //         msg.probabilities.push_back(1. / ((double)msg.gaussians.size()));

    //     if (msg.gaussians.size() == 0)
    //         ROS_ERROR("No valid paths found!");
    // }

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