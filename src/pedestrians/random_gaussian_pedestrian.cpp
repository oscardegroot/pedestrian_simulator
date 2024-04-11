#include <pedestrians/random_gaussian_pedestrian.h>

#include <pedestrian_simulator/configuration.h>

RandomGaussianPedestrian::RandomGaussianPedestrian(const SpawnRandomizer &spawn_randomizer, int seed_mp)
    : GaussianPedestrian(Waypoint(0., 0.), 0., Waypoint(0., 0.), seed_mp)
{
    // cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_; // At initialization: define the start seed of this ped
    // if (CONFIG.single_scenario_ != -1)
    //     cur_seed_ += CONFIG.single_scenario_;

    spawn_randomizer_ = spawn_randomizer;
}

void RandomGaussianPedestrian::Reset()
{
    if (CONFIG.single_scenario_ == -1)
        cur_seed_++; // We increase the seed after every simulation, to keep the behavior the same during each simulation

    random_generator_.reset(new RosTools::RandomGenerator(cur_seed_));

    // Generate a new start/goal
    start_ = spawn_randomizer_.GenerateStart(random_generator_.get());
    goal_ = spawn_randomizer_.GenerateGoal(random_generator_.get());
    velocity_ = spawn_randomizer_.GenerateVelocity(random_generator_.get());

    // Set the dynamics
    angle = std::atan2(goal_.y - start_.y, goal_.x - start_.x);
    B = Eigen::Vector2d(
        std::cos(angle),
        std::sin(angle));

    Pedestrian::Reset();
}
