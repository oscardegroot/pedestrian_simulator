#include <pedestrians/social_forces_pedestrian.h>

#include <pedestrian_simulator/configuration.h>
#include <ros_tools/logging.h>
// #include <pedsim_original/ped_scene.h>
// #include <pedsim_original/ped_agent.h>
// #include <pedsim_original/ped_waypoint.h>

SocialForcesPedestrian::SocialForcesPedestrian(const SpawnRandomizer &spawn_randomizer, int seed_mp, Ped::Tscene *pedsim_scene)
    : Pedestrian(Waypoint(0., 0.), 0.), seed_mp_(seed_mp)
{
    spawn_randomizer_ = spawn_randomizer;
    ResetSeed();

    pedsim_scene_ = pedsim_scene;

    Reset();
}

/** @brief copy constructor */
SocialForcesPedestrian::SocialForcesPedestrian(const SocialForcesPedestrian &other)
    : Pedestrian(other.start_, other.velocity_), seed_mp_(other.seed_mp_)
{
    id_ = other.id_;
    spawn_randomizer_ = other.spawn_randomizer_;
    cur_seed_ = other.cur_seed_;
    random_generator_.reset(new RosTools::RandomGenerator(*other.random_generator_));
    robot_state_ = other.robot_state_;

    position_ = other.position_;
    twist_ = other.twist_;
    start_ = other.start_;
    goal_ = other.goal_;
    noisy_twist_ = other.noisy_twist_;
    velocity_ = other.velocity_;
}

void SocialForcesPedestrian::LoadOtherPedestrians(std::vector<std::unique_ptr<Pedestrian>> *other_peds) { other_peds_ = other_peds; };

void SocialForcesPedestrian::LoadRobot(RobotState *robot_state) { robot_state_ = robot_state; };
void SocialForcesPedestrian::PreUpdateComputations(const double dt)
{
    other_ped_force_ = Eigen::Vector2d(0., 0.);
    for (auto &ped : *other_peds_)
    {
        if (ped->id_ == id_)
            continue;

        AddRepulsivePedestrianForce(other_ped_force_, *ped, dt);
    }
}

void SocialForcesPedestrian::Update(const double dt)
{
    (void)dt;
    if (!CONFIG.static_)
    {
        position_.x = pedsim_agent_->getx();
        position_.y = pedsim_agent_->gety();
        twist_(0) = pedsim_agent_->getVelocity().x;
        twist_(1) = pedsim_agent_->getVelocity().y;

        if (CONFIG.process_noise_.size() > 1 && CONFIG.process_noise_[0] > 1e-4) // If there is noise
        {
            double angle = std::atan2(twist_(1), twist_(0));

            Eigen::Vector2d noise = random_generator_->BivariateGaussian(Eigen::Vector2d(0., 0.),
                                                                         CONFIG.process_noise_[0],
                                                                         CONFIG.process_noise_[1],
                                                                         angle);
            // twist_(0) = pedsim_agent_->getVelocity().x + noise(0);
            // twist_(1) = pedsim_agent_->getVelocity().y + noise(1);
            position_.x = pedsim_agent_->getx() + noise(0) * CONFIG.delta_t_;
            position_.y = pedsim_agent_->gety() + noise(1) * CONFIG.delta_t_;

            pedsim_agent_->setPosition(position_.x, position_.y, pedsim_agent_->getz());
        }

        double dist_to_goal = RosTools::distance(
            Eigen::Vector2d(position_.x, position_.y),
            Eigen::Vector2d(goal_.x, goal_.y));

        if (dist_to_goal < 2.) // spawn_randomizer_.GetGoalRange() + 1.)
        {
            done_ = true;
            //     goal_ = GetGoal(goal_, spawn_randomizer_.GetMinTravelTime()); // spawn_randomizer_.GenerateGoal(random_generator_.get());
            //     pedsim_agent_->addWaypoint(new Ped::Twaypoint(goal_.x, goal_.y, spawn_randomizer_.GetGoalRange()));
            //     // pedsim_scene_->addWaypoint(pedsim_agent_, goal_.x, goal_.y);
        }
    }
}

void SocialForcesPedestrian::ResetSeed()
{
    cur_seed_ = seed_mp_ * 10000 + CONFIG.seed_; // At initialization: define the start seed of this ped
    if (CONFIG.single_scenario_ != -1)
        cur_seed_ += CONFIG.single_scenario_;
}

void SocialForcesPedestrian::Reset()
{
    done_ = false;
    cur_seed_++;
    random_generator_.reset(new RosTools::RandomGenerator(cur_seed_));

    start_ = spawn_randomizer_.GenerateStart(random_generator_.get());
    // velocity_ = spawn_randomizer_.GenerateVelocity(random_generator_.get());
    pedsim_agent_ = new Ped::Tagent();
    velocity_ = pedsim_agent_->getvmax(); // Determined by libpedsim

    goal_ = GetGoal(start_, spawn_randomizer_.GetMinTravelTime());

    // Ped::Twaypoint *w1 = new Ped::Twaypoint(start_.x, start_.y, spawn_randomizer_.GetGoalRange());
    Ped::Twaypoint *w2 = new Ped::Twaypoint(goal_.x, goal_.y, spawn_randomizer_.GetGoalRange());
    // pedsim_agent_->addWaypoint(w1);
    pedsim_agent_->addWaypoint(w2);

    pedsim_agent_->setPosition(start_.x, start_.y, 0.);
    pedsim_agent_->setRadius(CONFIG.ped_radius_);

    // velocity_ = pedsim_agent_->getvmax(); // Determined by libpedsim

    pedsim_scene_->addAgent(pedsim_agent_);

    Pedestrian::Reset();
}

Waypoint SocialForcesPedestrian::GetGoal(Waypoint start, double min_travel_time)
{
    (void)start;
    (void)min_travel_time;
    // int wall_select = random_generator_->Int(3);

    // double x, y;
    // if (wall_select < 2)
    // {
    //     x = (wall_select % 2 == 0 ? 26.0 : 2.);
    //     y = random_generator_->Double() * 24. + 2.;
    // }
    // else
    // {
    //     y = (wall_select % 2 == 0 ? 26.0 : 2.);
    //     x = random_generator_->Double() * 24. + 2.;
    // }
    // return Waypoint(x, y);

    Waypoint goal = spawn_randomizer_.GenerateGoal(random_generator_.get());
    Waypoint best_goal;
    double best_dist = 0.;
    for (int i = 0; start_.Distance(goal) < velocity_ * spawn_randomizer_.GetMinTravelTime() && i < 1000; i++) // Make sure the goal is far enough away (20s)
    {
        if (start_.Distance(goal) > best_dist)
        {
            best_dist = start_.Distance(goal);
            best_goal = goal;
        }

        goal = spawn_randomizer_.GenerateGoal(random_generator_.get());

        if (i == 999)
        {
            // LOG_ERROR("Could not find a goal far enough away from the start position!");
            return best_goal;
        }
    }
    return goal;
}

bool SocialForcesPedestrian::AddGoalForce(Eigen::Vector2d &force)
{
    if (position_.Distance(goal_) < 1.05 * velocity_ * CONFIG.delta_t_)
        return true; // Goal reached!

    double direction = position_.Angle(goal_);
    force += (Eigen::Vector2d(std::cos(direction) * velocity_, std::sin(direction) * velocity_) -
              Eigen::Vector2d(twist_(0), twist_(1))) /
             0.5;

    return false;
}

void SocialForcesPedestrian::AddRepulsivePedestrianForce(Eigen::Vector2d &force, Pedestrian &other_ped, const double dt)
{
    // force += GetRepulsiveForce(other_ped.GetPosition(), other_ped.GetSpeed(), dt);
    force += GetPedRepulsiveForce(other_ped.GetPosition(), other_ped.GetSpeed(), dt);
}

Eigen::Vector2d SocialForcesPedestrian::GetRepulsiveForce(const Eigen::Vector2d &other_pos, const Eigen::Vector2d &other_vel, const double dt)
{
    double l = CONFIG.social_l_;
    double A = CONFIG.social_strength_;
    double B = CONFIG.social_decay_; // A = 2.1, B = 0.3;
    Eigen::Vector2d r_ij(position_.x - other_pos(0), position_.y - other_pos(1));
    Eigen::Vector2d s_ij = (other_vel - GetSpeed()) * dt; // CONFIG.delta_t_;
    Eigen::Vector2d r_s_ij = r_ij - s_ij;

    double mod_r_ij, mod_r_s_ij, mod_s_ij;
    mod_r_ij = r_ij.norm();
    mod_r_s_ij = r_s_ij.norm();
    mod_s_ij = s_ij.norm();

    if (mod_r_ij < 1e-10 || mod_r_s_ij < 1e-10 || mod_s_ij < 1e-10)
        return Eigen::Vector2d(0., 0.);

    double b = std::sqrt(std::pow(mod_r_ij + mod_r_s_ij, 2.) - std::pow(mod_s_ij, 2.)) / 2.;
    // double b = std::sqrt(mod_std::pow(mod_r_ij + mod_r_s_ij, 2.) - std::pow(mod_s_ij, 2.)) / 2.;
    if (b < 1e-10)
        return Eigen::Vector2d(0., 0.);

    double g_pre_term = A * std::exp(-b / B) * (mod_r_ij + mod_r_s_ij) / (2. * b);
    Eigen::Vector2d g = g_pre_term * (r_ij / mod_r_ij + r_s_ij / mod_r_s_ij) / 2.;

    Eigen::Vector2d e_t_i(std::cos(position_.Angle(goal_)), std::sin(position_.Angle(goal_)));
    double cos_phi = (e_t_i(0) * r_ij(0) / mod_r_ij) + (e_t_i(1) * r_ij(1) / mod_r_ij);
    double w = l + (1. - l) * (1. + cos_phi) / 2.;

    return g * w;
}

Eigen::Vector2d SocialForcesPedestrian::GetPedRepulsiveForce(const Eigen::Vector2d &other_pos, const Eigen::Vector2d &other_vel, const double dt)
{
    (void)dt;
    // PEDSIM_ROS VERSION!
    // define relative importance of position vs velocity vector
    // (set according to Moussaid-Helbing 2009)
    //   const double lambdaImportance = 2.0;

    //   // define speed interaction
    //   // (set according to Moussaid-Helbing 2009)
    //   const double gamma = 0.35;

    //   // define speed interaction
    //   // (set according to Moussaid-Helbing 2009)
    //   const double n = 2;

    //   // define angular interaction
    //   // (set according to Moussaid-Helbing 2009)
    //   const double n_prime = 3;

    //   Tvector force;
    //   for (const Ped::Tagent* other : neighbors) {
    //     // don't compute social force to yourself
    //     if (other->id == id) continue;

    //     // compute difference between both agents' positions
    //     Tvector diff = other->p - p;

    //     if(other->getType() == ROBOT) diff /= robotPosDiffScalingFactor;

    //     Tvector diffDirection = diff.normalized();

    //     // compute difference between both agents' velocity vectors
    //     // Note: the agent-other-order changed here
    //     Tvector velDiff = v - other->v;

    //     // compute interaction direction t_ij
    //     Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
    //     double interactionLength = interactionVector.length();
    //     Tvector interactionDirection = interactionVector / interactionLength;

    //     // compute angle theta (between interaction and position difference vector)
    //     Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

    //     // compute model parameter B = gamma * ||D||
    //     double B = gamma * interactionLength;

    //     double thetaRad = theta.toRadian();
    //     double forceVelocityAmount =
    //         -exp(-diff.length() / B -
    //              (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
    //     double forceAngleAmount =
    //         -theta.sign() *
    //         exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

    //     Tvector forceVelocity = forceVelocityAmount * interactionDirection;
    //     Tvector forceAngle =
    //         forceAngleAmount * interactionDirection.leftNormalVector();

    //     force += forceVelocity + forceAngle;
    // parameters set according to Moussaid-Helbing 2009
    double A = 4.5;
    // relative importance of position vs velocity vector
    double lambdaImportance = 1.0;
    // speed interaction
    double gamma = 0.35;
    // speed interaction
    double n = 2;
    // angular interaction
    double n_prime = 3;

    double heading = std::atan2(twist_(1), twist_(0));
    // double c = std::cos(heading);
    // double s = std::sin(heading);
    // c, s = np.cos(current_heading), np.sin(current_heading)
    // R = np.array(((c, s), (-s, c)))

    auto R = RosTools::rotationMatrixFromHeading(heading);

    Eigen::Vector2d diff = other_pos - GetPosition();
    Eigen::Vector2d diff_ego = R * diff;
    // diff_ego = np.dot(R,diff)

    Eigen::Vector2d diff_dir = diff.normalized();

    // if np.linalg.norm(diff) != 0:
    //     diffDirection = diff / np.linalg.norm(diff)
    // else:
    //     diffDirection = np.zeros((2)) -> diff_dir = normalized version of diff
    Eigen::Vector2d diff_without_radius = diff - (0.5 + 0.5) * diff_dir;
    // diff_without_radius = diff - (radius +radius) * diffDirection
    diff_ego = diff_without_radius;
    // diff_ego = diff_without_radius /* Strange! */
    Eigen::Vector2d vel_diff = GetSpeed() - other_vel;
    Eigen::Vector2d vel_diff_ego = vel_diff;
    // velDiff = np.array(speed) - np.array(other_ped.vel_global_frame)
    // velDiff_ego = velDiff #np.dot(R,velDiff)
    /*
    def dotproduct(v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    def length(v):
        return math.sqrt(dotproduct(v, v))

    def angle(v1, v2):
        return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))*/

    Eigen::Vector2d interaction_vector = lambdaImportance * vel_diff_ego + diff_dir;
    double interaction_length = interaction_vector.norm();
    Eigen::Vector2d interaction_dir = interaction_vector.normalized();
    // if interactionLength != 0:
    //     interactionDirection = interactionVector / interactionLength
    // else:
    //     interactionDirection = np.zeros((2))

    auto &v = interaction_dir;
    auto &w = diff_dir;
    // double theta = std::acos(v(0)w / (v.norm() * w.norm()));
    // v = interactionDirection
    // w = diffDirection
    // theta = utils.calc_angle(v, w)  # np.arccos(v.dot(w)/(np.linalg.norm(v)*np.linalg.norm(w)))

    double inner = v(0) * w(0) + v(1) * w(1);
    double norms = v.norm() + w.norm();

    // inner = np.inner(v, w)
    // norms = np.linalg.norm(v) * np.linalg.norm(w)

    double cos;
    if (std::abs(norms) > 1e-8)
        cos = inner / norms;
    else
        cos = 0.;
    double theta = std::acos(std::max(-1., std::min(cos, 1.)));

    // if norms
    //     != 0 : cos = inner / norms else : cos = 0 theta = np.arccos(np.clip(cos, -1.0, 1.0))

    // v_norm = v / np.linalg.norm(v)
    // w_norm = w / np.linalg.norm(w)
    // theta = np.arccos(np.clip(np.dot(v_norm, w_norm), -1.0, 1.0))
    double B = gamma * interaction_length;
    // B = gamma *interactionLength
    double eps = 0.05;
    auto sign = [](double value)
    {
        return (0 < value) - (value < 0);
    };
    double forceFactorSocial = CONFIG.social_strength_; // parameter!
    double theta_rad = theta + B * eps * sign(forceFactorSocial);

    //  eps = 0.05 thetaRad = theta + B * eps * np.sign(self.forceFactorSocial) // math.radians(theta[0])
    /*
            def
            perpendicular(a) : b = np.empty_like(a)
                                       b[0] = -a[1] b[1] = a[0] return b*/

    // Eigen::Vector2d force(0., 0.);
    double force_velocity_magnitude = 0.;
    double force_angle_magnitude = 0.;
    if (diff_ego.norm() != 0 && vel_diff_ego.norm() != 0 && B != 0)
    {
        force_velocity_magnitude = -A * std::exp(-diff_ego.norm() / B - std::pow(n_prime * B * theta_rad, 2.));
        force_angle_magnitude = -A * sign(theta) * std::exp(-diff_ego.norm() / B - std::pow(n * B * theta_rad, 2.)); // Change sign to prefer left or right
        // -A * np.exp(-np.linalg.norm(diff_ego) / B - (n_prime * B * thetaRad) * (n_prime * B * thetaRad))
        // -A * np.sign(theta) * np.exp(-np.linalg.norm(diff_ego) / B - (n * B * thetaRad) * (n * B * thetaRad)) #change sign to prefer left or right
    }

    Eigen::Vector2d force_velocity = force_velocity_magnitude * interaction_dir;
    Eigen::Vector2d force_angle = force_angle_magnitude * interaction_dir.unitOrthogonal(); // May need "perpendicular"
    Eigen::Vector2d force = force_velocity + force_angle;
    return forceFactorSocial * force;
}