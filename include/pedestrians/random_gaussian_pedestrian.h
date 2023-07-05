#ifndef __RANDOM_GAUSSIAN_PEDESTRIAN_H__
#define __RANDOM_GAUSSIAN_PEDESTRIAN_H__

#include <pedestrians/gaussian_pedestrian.h>
#include <pedestrian_simulator/spawn_randomizer.h>

/** @brief Gaussian pedestrian spawning in a random location with a random goal */
class RandomGaussianPedestrian : public GaussianPedestrian
{

public:
    SpawnRandomizer spawn_randomizer_;

    RandomGaussianPedestrian(const SpawnRandomizer &spawn_randomizer, int seed_mp);

    virtual void Reset();
};
#endif // __RANDOM_GAUSSIAN_PEDESTRIAN_H__