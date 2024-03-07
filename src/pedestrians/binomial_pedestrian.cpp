#include <pedestrians/binomial_pedestrian.h>

#include <pedestrian_simulator/configuration.h>

BinomialPedestrian::BinomialPedestrian(const Waypoint &start, double velocity, int seed_mp)
    : Pedestrian(start, velocity), seed_mp_(seed_mp)
{
    cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_;

    p = CONFIG.p_binomial_;
    Reset();
}

void BinomialPedestrian::Update(const double dt)
{

    // BINOMIAL
    switch (state)
    {
    case PedState::STRAIGHT:

        twist_(0) = B_straight(0) * velocity_ * direction_;
        twist_(1) = B_straight(1) * velocity_ * direction_;

        // Transition
        if ((counter % 4 == 0) && (random_generator_->Double() <= p)) // Do this only once every 4 times
            state = PedState::CROSS;

        counter = (counter + 1) % 4;

        break;
    case PedState::CROSS:

        twist_(0) = B_cross(0) * velocity_ * direction_;
        twist_(1) = B_cross(1) * velocity_ * direction_;

        break;
    }

    Eigen::Vector2d process_noise_realization = random_generator_->BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                                     CONFIG.process_noise_[0],
                                                                                     CONFIG.process_noise_[1],
                                                                                     0.);

    noisy_twist_(0) = twist_(0) + process_noise_realization(0);
    noisy_twist_(1) = twist_(1) + process_noise_realization(1);

    // Update the position using the velocity and Gaussian process noise (and taking the frame into account)
    UpdatePosition(noisy_twist_(0), noisy_twist_(1), dt);
}

void BinomialPedestrian::Reset()
{
    cur_seed_++;
    random_generator_.reset(new RosTools::RandomGenerator(cur_seed_));

    Pedestrian::Reset();

    // Set the dynamics
    B_straight = Eigen::Vector2d(1., 0.);
    B_cross = Eigen::Vector2d(1.0 / std::sqrt(2), 1.0 / std::sqrt(2));
    state = PedState::STRAIGHT;
    counter = 0;

    // SH-MPC EXPERIMENTS!
    // direction_ = start_.x < 6. ? 1. : -1.;

    // PRIUS DRIVING
    direction_ = start_.y > 0 ? -1 : 1;
}