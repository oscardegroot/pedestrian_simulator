#ifndef __SOCIAL_FORCES_PEDESTRIAN_H__
#define __SOCIAL_FORCES_PEDESTRIAN_H__

#include <pedestrians/pedestrian.h>

#include <pedestrian_simulator/spawn_randomizer.h>

// #include <algorithm>
#include <pedsim_original/ped_includes.h>

class SocialForcesPedestrian : public Pedestrian
{
public:
    SocialForcesPedestrian(const SpawnRandomizer &spawn_randomizer, int seed_mp, Ped::Tscene *pedsim_scene);

    /** @brief copy constructor */
    SocialForcesPedestrian(const SocialForcesPedestrian &other);

    /** @brief Link to other pedestrians to move around them */
    virtual void LoadOtherPedestrians(std::vector<std::unique_ptr<Pedestrian>> *other_peds);

    /** @brief Link to the robot to move around it */
    virtual void LoadRobot(RobotState *robot_state);

public:
    // Compute the forces of other pedestrians before we update their position
    virtual void PreUpdateComputations(const double dt);

    virtual void Update(const double dt);

    virtual void Reset();
    virtual void ResetSeed();

protected:
    bool AddGoalForce(Eigen::Vector2d &force);

    void AddRepulsivePedestrianForce(Eigen::Vector2d &force, Pedestrian &other_ped, const double dt);
    Eigen::Vector2d GetRepulsiveForce(const Eigen::Vector2d &other_pos, const Eigen::Vector2d &other_vel, const double dt);
    Eigen::Vector2d GetPedRepulsiveForce(const Eigen::Vector2d &other_pos, const Eigen::Vector2d &other_vel, const double dt);

    Waypoint GetGoal(Waypoint start, double min_travel_time);
    std::unique_ptr<RosTools::RandomGenerator> random_generator_;
    int seed_mp_, cur_seed_;

    SpawnRandomizer spawn_randomizer_;
    std::vector<std::unique_ptr<Pedestrian>> *other_peds_;
    Eigen::Vector2d other_ped_force_;
    RobotState *robot_state_;

    Ped::Tscene *pedsim_scene_;
    Ped::Tagent *pedsim_agent_;
};
#endif // __SOCIAL_FORCES_PEDESTRIAN_H__