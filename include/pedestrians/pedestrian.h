#ifndef __PEDESTRIAN_H__
#define __PEDESTRIAN_H__

#include <pedestrian_simulator/types.h>

#include <geometry_msgs/Twist.h>

enum class PedState
{
    STRAIGHT = 0,
    CROSS
};

/** @brief Pedestrian base class */
class Pedestrian
{
public:
    Pedestrian(const Waypoint &start, double velocity);

public:
    virtual void Reset();

    // If computations should happen before we update the positions of all pedestrians
    virtual void PreUpdateComputations();
    virtual void PreUpdateComputations(const double dt);

    virtual void Update();
    virtual void Update(const double dt);

    virtual void UpdatePosition(const double vx, const double vy);
    virtual void UpdatePosition(const double vx, const double vy, const double dt);

    virtual Eigen::Vector2d GetSpeed() const;
    virtual Eigen::Vector2d GetPosition() const;

    unsigned int id_;

    Waypoint start_, goal_;
    Waypoint position_;

    geometry_msgs::Twist twist_;
    geometry_msgs::Twist noisy_twist_;

    double velocity_; // This is the preferred velocity
};

#endif // __PEDESTRIAN_H__
