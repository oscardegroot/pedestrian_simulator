#include <pedestrian_simulator/pedsim_manager.h>

#include <pedestrian_simulator/configuration.h>
#include <pedestrian_simulator/types.h>
#include <pedsim_original/ped_includes.h>

PedsimManager::PedsimManager(std::vector<StaticObstacle> &static_obstacles)

{
    static_obstacles_ = &static_obstacles;

    pedsim_scene_ = new Ped::Tscene(-200, -200, 400, 400);

    Init();
}

PedsimManager::~PedsimManager()
{
    pedsim_scene_->clear();
    delete pedsim_scene_;
}

void PedsimManager::Update(double dt) { pedsim_scene_->moveAgents(dt); }

void PedsimManager::Init()
{
    AddObstacles();

    if (CONFIG.interaction_)
        AddRobot();
}

void PedsimManager::Reset()
{
    Clear();
    Init();
}

void PedsimManager::Clear()
{
    pedsim_scene_->clear();
    pedsim_robot_ = nullptr;
}

void PedsimManager::AddObstacles()
{
    for (auto &obs : *static_obstacles_)
    {
        Ped::Tobstacle *o = new Ped::Tobstacle(obs.min_x, obs.min_y, obs.max_x, obs.max_y);
        pedsim_scene_->addObstacle(o);
    }
}

/** @todo types for robot and pedestrians */
Ped::Tagent *PedsimManager::AddAgent(double start_x, double start_y,
                                     double goal_x, double goal_y)
{
    Ped::Tagent *agent = new Ped::Tagent();
    // Ped::Twaypoint *w1 = new Ped::Twaypoint(start_x, start_y, 3.);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(goal_x, goal_y, 3.);
    // agent->addWaypoint(w1);
    agent->addWaypoint(w2);

    agent->setPosition(start_x, start_y, 0.);

    agent->setType((int)AgentType::PEDESTRIAN); // Default = 0.
    agent->setRadius(CONFIG.ped_radius_);
    pedsim_scene_->addAgent(agent);

    return agent;
}

/** @todo types for robot and pedestrians */
// Ped::Tagent *PedsimManager::addGoal(Ped::Tagent *agent, double goal_x, double goal_y)
// {
//     auto waypoints = agent->getWaypoints();

//     agent->clearWaypoints();

//     Ped::Twaypoint *w1 = waypoints.back();
//     Ped::Twaypoint *w2 = new Ped::Twaypoint(goal_x, goal_y, 5.);
//     agent->addWaypoint(w1);
//     agent->addWaypoint(w2);

//     return agent;
// }

void PedsimManager::AddRobot()
{
    pedsim_robot_ = new Ped::Tagent();

    Ped::Twaypoint *w1 = new Ped::Twaypoint(0., 0., 5.);
    // Ped::Twaypoint *w2 = new Ped::Twaypoint(25., 0., 5.);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(26., 26., 5.);
    pedsim_robot_->addWaypoint(w1);
    pedsim_robot_->addWaypoint(w2);

    pedsim_robot_->setType((int)AgentType::ROBOT); // 1 = robot (not sure if needed)
    pedsim_robot_->setRadius(0.325);
    pedsim_scene_->addAgent(pedsim_robot_);
}

void PedsimManager::SetRobotPosition(const double x, const double y)
{
    if (pedsim_robot_)
        pedsim_robot_->setPosition(x, y, 0.);
}
void PedsimManager::SetRobotVelocity(const double vx, const double vy)
{
    if (pedsim_robot_)
        pedsim_robot_->setVelocity(vx, vy, 0.);
}
const std::vector<Ped::Tagent *> &PedsimManager::GetAllAgents() const { return pedsim_scene_->getAllAgents(); }