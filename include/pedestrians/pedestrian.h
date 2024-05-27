#ifndef __PEDESTRIAN_H__
#define __PEDESTRIAN_H__

#include <pedestrian_simulator/types.h>

#include <ros_tools/random_generator.h>

#include <memory>

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
    virtual void ResetSeed(){};

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
    bool done_{false};

    Waypoint start_, goal_;
    Waypoint position_;

    Eigen::Vector2d twist_;
    Eigen::Vector2d noisy_twist_;

    double velocity_; // This is the preferred velocity
};

#endif // __PEDESTRIAN_H__
