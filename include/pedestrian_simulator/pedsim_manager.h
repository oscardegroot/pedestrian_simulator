#ifndef __PEDSIM_MANAGER_H__
#define __PEDSIM_MANAGER_H__

/**
 * @file pedsim_manager.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Uses libpedsim to simulate pedestrians. Can be used to predict the motion as well.
 * @version 0.1
 * @date 2023-07-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <vector>

// Forward declaration
namespace Ped
{
    class Tagent;
    class Tscene;
};
class StaticObstacle;

enum class AgentType
{
    PEDESTRIAN = 0,
    ROBOT = 1
};

class PedsimManager
{
public:
    PedsimManager(std::vector<StaticObstacle> &obstacles);
    virtual ~PedsimManager();

public:
    void Update(double dt); /** @brief Move all agents with timestep dt */

    void Reset(); /** @brief Reset the scene */
    void Clear(); /** @brief Clear all data */

    /** @brief Add an agent to the scene, will load standard values from the config */
    Ped::Tagent *AddAgent(double start_x, double start_y,
                          double goal_x, double goal_y);

    /** @brief If the pedestrians should interact with the robot, add a robot */
    void AddRobot();
    void SetRobotPosition(const double x, const double y);
    void SetRobotVelocity(const double vx, const double vy);

    Ped::Tscene *GetScene() const { return pedsim_scene_; }
    Ped::Tagent *GetRobot() const { return pedsim_robot_; }

    const std::vector<Ped::Tagent *> &GetAllAgents() const;

private:
    void Init();
    void AddObstacles(); /** @todo Integrate with xml */

private:
    Ped::Tscene *pedsim_scene_;
    Ped::Tagent *pedsim_robot_;

    std::vector<StaticObstacle> *static_obstacles_;
};

#endif // __PEDSIM_MANAGER_H__