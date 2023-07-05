#ifndef __BINOMIAL_PEDESTRIAN_H__
#define __BINOMIAL_PEDESTRIAN_H__

#include <pedestrians/pedestrian.h>

/** @brief Pedestrian that may cross with a probability, in the binomial sense */
class BinomialPedestrian : public Pedestrian
{
public:
    BinomialPedestrian(const Waypoint &start, double velocity, int seed_mp);

public:
    Eigen::Vector2d B_straight;
    Eigen::Vector2d B_cross;
    PedState state;
    double p;
    int counter;
    std::unique_ptr<RosTools::RandomGenerator> random_generator_;
    int seed_mp_, cur_seed_;

    virtual void Update(const double dt) override;

public:
    virtual void Reset();

    int direction_;
};
#endif // __BINOMIAL_PEDESTRIAN_H__