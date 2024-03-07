#include <pedestrians/gaussian_pedestrian.h>

#include <pedestrian_simulator/configuration.h>

GaussianPedestrian::GaussianPedestrian(const Waypoint &start, double velocity, const Waypoint &end, int seed_mp)
    : Pedestrian(start, velocity), seed_mp_(seed_mp)
{

    cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_; // At initialization: define the start seed of this ped
    if (CONFIG.single_scenario_ != -1)
        cur_seed_ += CONFIG.single_scenario_;
    goal_ = end;
    Reset();

    // Set the dynamics
    angle = std::atan2(goal_.y - start.y, goal_.x - start.x);
    B = Eigen::Vector2d(
        std::cos(angle),
        std::sin(angle));
}

void GaussianPedestrian::Update(const double dt)
{

    Eigen::Vector2d process_noise_realization = random_generator_->BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                                     CONFIG.process_noise_[0],
                                                                                     CONFIG.process_noise_[1],
                                                                                     angle);

    twist_(0) = B(0) * velocity_;
    twist_(1) = B(1) * velocity_;
    noisy_twist_ = twist_;

    // Major / minor -> Cov = [major^2, 0; 0 minor^2]
    if (!CONFIG.static_) // If static, do not update the position
    {
        noisy_twist_(0) += process_noise_realization(0);
        noisy_twist_(1) += process_noise_realization(1);
        UpdatePosition(noisy_twist_(0), noisy_twist_(1), dt);
    }

    // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
}

void GaussianPedestrian::Reset()
{

    cur_seed_++; // We increase the seed after every simulation, to keep the behavior the same during each simulation
    random_generator_.reset(new RosTools::RandomGenerator(cur_seed_));

    Pedestrian::Reset();
}