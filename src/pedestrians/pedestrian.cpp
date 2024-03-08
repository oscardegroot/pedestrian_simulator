#include <pedestrians/pedestrian.h>

#include <pedestrian_simulator/configuration.h>
#include <Eigen/Dense>

Pedestrian::Pedestrian(const Waypoint &start, double velocity)
    : velocity_(velocity)
{
    start_ = start;
    Reset();
}

void Pedestrian::Reset()
{
    position_ = start_;
}

// If computations should happen before we update the positions of all pedestrians
void Pedestrian::PreUpdateComputations() { PreUpdateComputations(CONFIG.delta_t_); }
void Pedestrian::PreUpdateComputations(const double dt) { (void)dt; };
void Pedestrian::Update() { Update(CONFIG.delta_t_); }
void Pedestrian::Update(const double dt) { (void)dt; };
void Pedestrian::UpdatePosition(const double vx, const double vy) { UpdatePosition(vx, vy, CONFIG.delta_t_); }
void Pedestrian::UpdatePosition(const double vx, const double vy, const double dt)
{
    // Rotate movement in the frame
    Eigen::Vector2d rotated_delta_p = CONFIG.origin_R_ * Eigen::Vector2d(vx, vy) * dt; // CONFIG.delta_t_;

    position_.x += rotated_delta_p(0);
    position_.y += rotated_delta_p(1);
}

Eigen::Vector2d Pedestrian::GetSpeed() const { return Eigen::Vector2d(twist_(0), twist_(1)); };
Eigen::Vector2d Pedestrian::GetPosition() const { return Eigen::Vector2d(position_.x, position_.y); };