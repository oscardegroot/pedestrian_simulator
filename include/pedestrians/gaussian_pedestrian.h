#ifndef __GAUSSIAN_PEDESTRIAN_H__
#define __GAUSSIAN_PEDESTRIAN_H__

#include <pedestrians/pedestrian.h>

/** @brief Pedestrian that moves under Gaussian process noise */
class GaussianPedestrian : public Pedestrian
{
public:
    GaussianPedestrian(const Waypoint &start, double velocity, const Waypoint &end, int seed_mp);

public:
    double angle;
    Eigen::Vector2d B;
    std::unique_ptr<RosTools::RandomGenerator> random_generator_;
    int cur_seed_, seed_mp_;

    virtual void Update(const double dt) override;

public:
    virtual void Reset();
};
#endif // __GAUSSIAN_PEDESTRIAN_H__