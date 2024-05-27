#include "pedestrian_simulator/pedestrian_simulator.h"

#include <pedestrian_simulator/configuration.h>

#include <pedestrians/pedestrian.h>
#include <pedestrians/gaussian_pedestrian.h>
#include <pedestrians/binomial_pedestrian.h>
#include <pedestrians/random_gaussian_pedestrian.h>
#include <pedestrians/social_forces_pedestrian.h>

#include <ros_tools/convertions.h>
#include <ros_tools/logging.h>

PedestrianSimulator::PedestrianSimulator()
{
    LOG_INFO("Initializing");

    Config::Get().Init();

    ReadScenario();

    Reset();

    for (size_t i = 0; i < colors_.size(); i++)
        colors_[i] /= 256.;

    LOG_SUCCESS("Ready");
}

void PedestrianSimulator::ReadScenario()
{
    xml_reader_.reset(new XMLReader());
    random_generator_ = RosTools::RandomGenerator(CONFIG.seed_);

    // Read pedestrian data
    switch (CONFIG.ped_type_)
    {
    case PedestrianType::WAYPOINT:
        xml_reader_->GetPedestrians(pedestrians_);
        break;
    case PedestrianType::GAUSSIAN:

        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            if (xml_reader_->is_random_[ped_id])
            {
                int random_select = random_generator_.Int(xml_reader_->spawn_randomizers_.size() - 1);
                pedestrians_.back().reset(new RandomGaussianPedestrian(xml_reader_->spawn_randomizers_[random_select], ped_id));
            }
            else
            {
                pedestrians_.back().reset(new GaussianPedestrian(xml_reader_->pedestrians_[ped_id]->start_, CONFIG.ped_velocity_, xml_reader_->pedestrians_[ped_id]->goal_, ped_id));
            }
        }
        break;
    case PedestrianType::BINOMIAL:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new BinomialPedestrian(xml_reader_->pedestrians_[ped_id]->start_, CONFIG.ped_velocity_, ped_id));
        }
        break;
    case PedestrianType::SOCIAL:
        pedsim_manager_.reset(new PedsimManager(xml_reader_->static_obstacles_)); // Initialize the libpedsim backend

        pedsim_prediction_manager_.reset(new PedsimManager(xml_reader_->static_obstacles_)); // Initialize the libpedsim backend
        LOG_INFO("Pedestrian Simulator: Spawning social forces pedestrian");
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            int random_select = random_generator_.Int(xml_reader_->spawn_randomizers_.size() - 1);
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new SocialForcesPedestrian(xml_reader_->spawn_randomizers_[random_select], ped_id, pedsim_manager_->GetScene()));
        }

        for (auto &ped : pedestrians_)
        {
            ((SocialForcesPedestrian *)(ped.get()))->LoadOtherPedestrians(&pedestrians_);
            ((SocialForcesPedestrian *)(ped.get()))->LoadRobot(&robot_state_);
        }
        break;
    }

    for (size_t i = 0; i < pedestrians_.size(); i++) // Assign IDs for all pedestrians
        pedestrians_[i]->id_ = i;
}

void PedestrianSimulator::ResetToStart()
{
    for (auto &ped : pedestrians_)
    {
        // Reset pedestrians to their starting position
        ped->ResetSeed();
    }
}

void PedestrianSimulator::Reset()
{
    if (pedsim_manager_)
        pedsim_manager_->Reset();

    for (auto &ped : pedestrians_)
    {
        // Reset pedestrians to their starting position
        ped->Reset();
    }

    // Check if the start/end point is in collision and regenerate if necessary
    if (CONFIG.collision_free_spawn_)
    {
        for (auto &ped : pedestrians_)
        {
            for (auto &other_ped : pedestrians_) // We fix the other pedestrian so we do not change the peds in the outer loop
            {
                while (ped->id_ != other_ped->id_ && ped->start_.Distance(other_ped->start_) < CONFIG.ped_radius_ * 2)
                    other_ped->Reset();
            }
        }
    }
}

void PedestrianSimulator::Loop()
{
    // Moving pedsim agents
    if (pedsim_manager_)
        pedsim_manager_->Update(CONFIG.delta_t_);

    if (CONFIG.debug_output_)
        LOG_INFO("PedestrianSimulator: Update");

    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
        ped->PreUpdateComputations();

    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
    {
        ped->Update();
        // ped->MoveFrame(vehicle_speed_);
    }

    PublishDebugVisuals();

    VisualizeRobot();

    VisualizePedestrians();

    VisualizeStaticObstacles();
}

std::vector<Prediction> PedestrianSimulator::GetPedestrians()
{
    std::vector<Prediction> predictions;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        predictions.emplace_back();
        auto &prediction = predictions.back();
        prediction.id = id;

        prediction.Add(
            Eigen::Vector2d(ped->position_.x,
                            ped->position_.y),
            std::atan2(ped->twist_(1), ped->twist_(0)),
            ped->noisy_twist_);

        id++;
    }

    return predictions;
}

std::vector<Prediction> PedestrianSimulator::GetPredictions()
{
    // Decide how to compute predictions
    switch (CONFIG.ped_type_)
    {
    case PedestrianType::GAUSSIAN:
        return GetGaussianPredictions();
    // case PedestrianType::BINOMIAL:
    // PublishBinomialPredictions();
    // return;
    case PedestrianType::SOCIAL:
        if (CONFIG.constant_velocity_predictions_)
            return GetGaussianPredictions();
        else
            return GetSocialPredictions(); // With libpedsim
    default:

        return GetGaussianPredictions();
    }

    // Otherwise we use this function
    /*
        mpc_planner_msgs::obstacle_array prediction_array;
        prediction_array.header.frame_id = "map";

        // Idea: Copy the pedestrian and step all of them one by one recording their positions then
        std::vector<std::unique_ptr<Pedestrian>> copied_pedestrians;
        std::vector<mpc_planner_msgs::gaussian> gaussian_msgs;

        RobotState copied_robot = robot_state_;

        size_t id = 0;
        for (auto &ped : pedestrians_)
        {
            SocialForcesPedestrian &cur_ped = *((SocialForcesPedestrian *)(ped.get()));

            // Copy the pedestrian here
            copied_pedestrians.push_back(nullptr);
            copied_pedestrians.back().reset(new SocialForcesPedestrian(cur_ped));
            SocialForcesPedestrian *copied_ped = (SocialForcesPedestrian *)(copied_pedestrians.back().get());

            copied_ped->LoadOtherPedestrians(&copied_pedestrians); // Link this pedestrian to the copied pedestrians
            copied_ped->LoadRobot(&copied_robot);

            mpc_planner_msgs::obstacle_gmm gmm_msg;
            gmm_msg.id = id;

            // Initial position
            gmm_msg.pose.position.x = copied_ped->position_.x - vehicle_frame_.position.x;
            gmm_msg.pose.position.y = copied_ped->position_.y - vehicle_frame_.position.y;
            gmm_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(copied_ped->twist_(1), copied_ped->twist_(0)));

            prediction_array.obstacles.push_back(gmm_msg);

            gaussian_msgs.emplace_back();
            id++;
        }

        for (int k = 0; k < CONFIG.horizon_N_; k++)
        {
            copied_robot.pos += copied_robot.vel * CONFIG.delta_t_; // Update the robot position assuming constant velocity

            for (auto &ped : copied_pedestrians)
                ped->PreUpdateComputations(CONFIG.prediction_step_);

            id = 0;
            for (auto &ped : copied_pedestrians)
            {
                ped->Update(CONFIG.prediction_step_);

                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = ped->position_.x;
                pose.pose.position.y = ped->position_.y;
                pose.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->twist_(1), ped->twist_(0)));

                gaussian_msgs[id].mean.poses.push_back(pose);

                // The variance is simply the uncertainty per stage
                gaussian_msgs[id].major_semiaxis.push_back(CONFIG.process_noise_[0]);
                gaussian_msgs[id].minor_semiaxis.push_back(CONFIG.process_noise_[1]);

                id++;
            }
        }

        for (size_t id = 0; id < copied_pedestrians.size(); id++)
        {
            prediction_array.obstacles[id].gaussians.push_back(gaussian_msgs[id]);
            prediction_array.obstacles[id].probabilities.push_back(1.0);
        }
        prediction_array.header.stamp = ros::Time::now();
        obstacle_trajectory_prediction_pub_.publish(prediction_array);*/
}

std::vector<Prediction> PedestrianSimulator::GetSocialPredictions()
{
    std::vector<Prediction> predictions;

    // Build up the scene
    pedsim_prediction_manager_->Reset();

    for (auto &pedestrian : pedestrians_)
    {
        if (pedestrian->done_) // Agents that reached their goal dissappear
            continue;

        Ped::Tagent *a = pedsim_prediction_manager_->AddAgent(pedestrian->GetPosition()(0),
                                                              pedestrian->GetPosition()(1),
                                                              pedestrian->goal_.x,
                                                              pedestrian->goal_.y);

        a->setVelocity(pedestrian->GetSpeed()(0), pedestrian->GetSpeed()(1), 0.);
        a->setVmax(pedestrian->velocity_);
    }

    size_t id = 0;
    for (auto &ped : pedsim_prediction_manager_->GetAllAgents())
    {
        if (ped->gettype() == (int)AgentType::ROBOT)
            continue;

        predictions.emplace_back();
        auto &prediction = predictions.back();

        prediction.id = id;

        // Initial position
        prediction.Add(
            Eigen::Vector2d(ped->getx(),
                            ped->gety()),
            std::atan2(ped->getvy(), ped->getvx()),
            Eigen::Vector2d(ped->getvx(), ped->getvy()));

        id++;
    }

    for (int k = 0; k < CONFIG.horizon_N_; k++)
    {
        id = 0;
        pedsim_prediction_manager_->Update(CONFIG.prediction_step_);

        for (auto &ped : pedsim_prediction_manager_->GetAllAgents())
        {
            if (ped->gettype() == (int)AgentType::ROBOT)
                continue;

            predictions[id].Add(
                Eigen::Vector2d(ped->getx(), ped->gety()),
                std::atan2(ped->getvy(), ped->getvx()),
                Eigen::Vector2d(ped->getvx(), ped->getvy()),
                CONFIG.process_noise_[0],
                CONFIG.process_noise_[1]);

            id++;
        }
    }

    return predictions;
}

std::vector<Prediction> PedestrianSimulator::GetGaussianPredictions()
{
    std::vector<Prediction> predictions;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        if (ped->done_)
            continue;

        predictions.emplace_back();
        auto &prediction = predictions.back();
        prediction.id = id;

        Eigen::Vector2d pos(ped->position_.x, ped->position_.y);

        // Add the current position as first prediction
        prediction.Add(pos,
                       std::atan2(ped->twist_(1), ped->twist_(0)),
                       ped->noisy_twist_,
                       CONFIG.process_noise_[0],
                       CONFIG.process_noise_[1]);

        for (int k = 0; k < CONFIG.horizon_N_; k++) // 1 - N
        {
            Eigen::Vector2d rotated_predict = CONFIG.origin_R_ * Eigen::Vector2d(ped->twist_(0), ped->twist_(1));

            pos(0) += rotated_predict(0) * CONFIG.prediction_step_;
            pos(1) += rotated_predict(1) * CONFIG.prediction_step_;

            prediction.Add(pos,
                           std::atan2(ped->twist_(1), ped->twist_(0)),
                           ped->noisy_twist_,
                           CONFIG.process_noise_[0],
                           CONFIG.process_noise_[1]);
        }
        id++;
    }

    return predictions;
}

void PedestrianSimulator::PublishDebugVisuals()
{
    auto &debug_visuals = VISUALS.getPublisher("debug");
    RosTools::ROSPointMarker &arrow = debug_visuals.getNewPointMarker("ARROW");
    arrow.setScale(0.8 * 1.5, 0.15 * 1.5, 0.15 * 1.5);

    RosTools::ROSPointMarker &start_goal_marker = debug_visuals.getNewPointMarker("SPHERE"); // Plot the start and goal locations of the pedestrians
    start_goal_marker.setScale(0.4, 0.4, 0.4);

    RosTools::ROSPointMarker &robot_position = debug_visuals.getNewPointMarker("Cube");
    robot_position.setScale(0.4, 0.4, 0.05);
    robot_position.setColorInt(3, 1., RosTools::Colormap::BRUNO);

    for (auto &ped : pedestrians_)
    {
        arrow.setColor(1.0, 0.0, 0.0, 1.);

        // Eigen::Matrix3d H;
        // H.block(0, 0, 2, 2) = RosTools::rotationMatrixFromHeading(vehicle_frame_.orientation.z);
        // H.block(2, 0, 2, 1) = Eigen::Vector2d(vehicle_frame_.position.x, vehicle_frame_.position.y);
        // H(2, 2) = 1.0;
        // Eigen::Vector3d pr = H * Eigen::Vector3d(ped->position_.x, ped->position_.y, 1);
        // arrow.setColorInt(15);
        // arrow.setColor(247. / 256., 177. / 256., 64. / 256.);

        arrow.setOrientation(std::atan2(ped->twist_(1), ped->twist_(0)));

        arrow.addPointMarker(Eigen::Vector3d(ped->position_.x, ped->position_.y, 0.));

        // start_goal_marker.setColor(1., 0., 0.);
        // start_goal_marker.addPointMarker(Eigen::Vector3d(ped->start_.x, ped->start_.y, 0.));

        // start_goal_marker.setColor(0., 1., 0.);
        // start_goal_marker.addPointMarker(Eigen::Vector3d(ped->goal_.x, ped->goal_.y, 0.));
    }

    robot_position.addPointMarker(Eigen::Vector3d(robot_state_.pos(0), robot_state_.pos(1), 0.));

    debug_visuals.publish();
}

void PedestrianSimulator::SetRobotState(const RobotState &state)
{
    robot_state_ = state;
    if (pedsim_manager_)
    {
        pedsim_manager_->SetRobotPosition(robot_state_.pos(0), robot_state_.pos(1));
        pedsim_manager_->SetRobotVelocity(robot_state_.vel(0), robot_state_.vel(1));
    }
}

void PedestrianSimulator::VisualizeRobot()
{
    if ((!pedsim_manager_) || pedsim_manager_->GetRobot() == nullptr)
        return;

    auto &robot_visual = VISUALS.getPublisher("robot_position");
    RosTools::ROSPointMarker &obstacle_marker = robot_visual.getNewPointMarker("CYLINDER");
    obstacle_marker.setColorInt(0, 1.);
    obstacle_marker.setScale(pedsim_manager_->GetRobot()->getradius() * 2,
                             pedsim_manager_->GetRobot()->getradius() * 2,
                             0.1);

    obstacle_marker.addPointMarker(Eigen::Vector3d(pedsim_manager_->GetRobot()->getx(),
                                                   pedsim_manager_->GetRobot()->gety(), 0.05));

    robot_visual.publish();
}

void PedestrianSimulator::VisualizePedestrians()
{
    auto &ped_visual = VISUALS.getPublisher("visualization");
    auto &ped_model = ped_visual.getNewModelMarker();

    for (auto &ped : pedestrians_) // Publish a pedestrian model
    {
        if (ped->done_)
            continue;
        Eigen::Vector2d rotated_twist = CONFIG.origin_R_ * Eigen::Vector2d(ped->twist_(0), ped->twist_(1));
        double angle = std::atan2(rotated_twist(1), rotated_twist(0));
        ped_model.setOrientation(RosTools::angleToQuaternion(angle + M_PI_2));

        // Consistent with LMPCC
        // int select = ped->id_;
        // select %= (int)(colors_.size() / 3); // We only have 20 values
        // select = (int)(colors_.size() / 3) - 1 - select;
        // ped_model.setColor(colors_[3 * select + 0], colors_[3 * select + 1], colors_[3 * select + 2], 1.0);
        ped_model.setColorInt(ped->id_, 1.0, RosTools::Colormap::BRUNO);

        ped_model.addPointMarker(Eigen::Vector3d(ped->position_.x, ped->position_.y, 0.));
    }
    ped_visual.publish();
}

void PedestrianSimulator::VisualizeStaticObstacles()
{
    auto &static_obstacle_visual = VISUALS.getPublisher("static_obstacles");
    RosTools::ROSPointMarker &obstacle_marker = static_obstacle_visual.getNewPointMarker("Cube");
    double min_dim = 0.5;
    obstacle_marker.setColor(0.1, 0.1, 0.1, 0.8);

    for (auto &obstacle : xml_reader_->static_obstacles_)
    {
        obstacle_marker.setScale(std::max(min_dim, obstacle.max_x - obstacle.min_x),
                                 std::max(min_dim, obstacle.max_y - obstacle.min_y),
                                 2.);
        obstacle_marker.addPointMarker(Eigen::Vector3d((obstacle.max_x + obstacle.min_x) / 2.,
                                                       (obstacle.max_y + obstacle.min_y) / 2.,
                                                       1.));
    }

    static_obstacle_visual.publish();
}