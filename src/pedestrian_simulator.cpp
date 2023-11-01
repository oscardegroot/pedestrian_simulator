#include "pedestrian_simulator/pedestrian_simulator.h"

#include <pedestrian_simulator/configuration.h>

#include <pedestrians/pedestrian.h>
#include <pedestrians/gaussian_pedestrian.h>
#include <pedestrians/binomial_pedestrian.h>
#include <pedestrians/random_gaussian_pedestrian.h>
#include <pedestrians/social_forces_pedestrian.h>
#include <pedestrians/waypoint_pedestrian.h>

#include <tf/tf.h>
#include <derived_object_msgs/Object.h>
#include <mpc_msgs/obstacle_array.h>
#include <mpc_msgs/obstacle_gmm.h>
#include <mpc_msgs/gaussian.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

PedestrianSimulator::PedestrianSimulator()
{
    ROS_INFO("PedestrianSimulator: Initializing");

    Config::Get().Init();

    debug_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "pedestrian_simulator/debug", "map", 50));
    static_obstacle_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "pedestrian_simulator/static_obstacles", "map", 50));
    robot_visual_.reset(new RosTools::ROSMarkerPublisher(nh_, "pedestrian_simulator/robot", "map", 5));

    ped_model_visuals_ = nh_.advertise<visualization_msgs::MarkerArray>("/pedestrian_simulator/visualization", 5);

    obstacle_pub_ = nh_.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    // obstacle_prediction_pub_ = nh_.advertise<mpc_msgs::obstacle_array>("/pedestrian_simulator/predictions", 1);
    obstacle_trajectory_prediction_pub_ = nh_.advertise<mpc_msgs::obstacle_array>("/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, &PedestrianSimulator::ResetCallback, this);
    // vehicle_speed_sub_ = nh_.subscribe("/lmpcc/vehicle_speed", 1, &PedestrianSimulator::VehicleVelocityCallback, this);
    robot_state_sub_ = nh_.subscribe("/robot_state", 1, &PedestrianSimulator::RobotStateCallback, this);

    setting_N_sub_ = nh_.subscribe("/pedestrian_simulator/N", 1, &PedestrianSimulator::SettingNCallback, this);
    setting_dt_sub_ = nh_.subscribe("/pedestrian_simulator/dt", 1, &PedestrianSimulator::SettingdtCallback, this);
    // setting_hz_sub_ = nh_.subscribe("/pedestrian_simulator/Hz", 1, &PedestrianSimulator::SettingHzCallback, this);

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
        ROS_INFO("Pedestrian Simulator: Spawning social forces pedestrian");
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

    // For Carla only!
    for (size_t ped_id = 0; ped_id < pedestrians_.size(); ped_id++)
    {
        carla_position_pub_.push_back(nh_.advertise<geometry_msgs::Pose>("/carla/simulated_peds/" + std::to_string(ped_id) + "/position", 1));
        carla_velocity_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("/carla/simulated_peds/" + std::to_string(ped_id) + "/velocity", 1));

        if (CONFIG.pretend_to_be_optitrack_)
            optitrack_publishers_.push_back(nh_.advertise<geometry_msgs::PoseStamped>("mocap_obstacle" + std::to_string(ped_id + 1) + "/pose", 1));
    }

    // The robot state publisher
    if (CONFIG.pretend_to_be_optitrack_)
    {
        optitrack_publishers_.push_back(nh_.advertise<geometry_msgs::PoseStamped>("Robot_1/pose", 1));
        joystick_publisher_ = nh_.advertise<sensor_msgs::Joy>("/bluetooth_teleop/joy", 1); // Disable the deadman switch
    }
    // Pick a path
    Reset();

    for (size_t i = 0; i < colors_.size(); i++)
        colors_[i] /= 256.;

    // Initialize the node loop
    timer_ = nh_.createTimer(ros::Duration(1.0 / CONFIG.update_frequency_), &PedestrianSimulator::Poll, this);

    if (CONFIG.use_path_origin_)
        path_origin_sub_ = nh_.subscribe("/roadmap/reference", 1, &PedestrianSimulator::OriginCallback, this);

    ROS_INFO("PedestrianSimulator: Ready");
}

void PedestrianSimulator::ResetCallback(const std_msgs::Empty &msg)
{
    // ROS_INFO("PedestrianSimulator: Reset callback");
    Reset();
}

void PedestrianSimulator::RobotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robot_state_ = RobotState(msg);
    // robot_state_.pos += robot_state_.vel * CONFIG.delta_t_; // Usually the robot state is lagging one time step behind

    if (pedsim_manager_)
    {
        pedsim_manager_->SetRobotPosition(robot_state_.pos(0), robot_state_.pos(1));
        pedsim_manager_->SetRobotVelocity(robot_state_.vel(0), robot_state_.vel(1));
    }
    // pedsim_robot_->setPosition(robot_state_.pos(0), robot_state_.pos(1), 0.);

    if (CONFIG.pretend_to_be_optitrack_)
    {
        geometry_msgs::PoseStamped robot_state_out_msg = *msg;
        robot_state_out_msg.pose.orientation = RosTools::angleToQuaternion(robot_state_out_msg.pose.orientation.z);
        optitrack_publishers_.back().publish(robot_state_out_msg); // Forward the message on the optitrack topic

        sensor_msgs::Joy joystick_msg;
        joystick_msg.axes.resize(3);
        joystick_msg.axes[2] = -1.;
        joystick_msg.header.stamp = ros::Time::now();
        joystick_publisher_.publish(joystick_msg);
    }
}

void PedestrianSimulator::VehicleVelocityCallback(const geometry_msgs::Twist &msg)
{

    vehicle_frame_.position.x += msg.linear.x * CONFIG.delta_t_;
    vehicle_frame_.position.y += msg.linear.y * CONFIG.delta_t_;
    vehicle_frame_.orientation.z += msg.angular.z * CONFIG.delta_t_; // angular velocity is stored in orientation / angular .z (euler not quaternion)
}

void PedestrianSimulator::OriginCallback(const nav_msgs::Path &msg)
{
    double angle = std::atan2(msg.poses[1].pose.position.y - msg.poses[0].pose.position.y, msg.poses[1].pose.position.x - msg.poses[0].pose.position.x);

    // Update if the path is new
    if (origin_.position.x != msg.poses[0].pose.position.x || origin_.position.y != msg.poses[0].pose.position.y || RosTools::rotationMatrixFromHeading(-angle) != CONFIG.origin_R_)
    {
        // We save the rotation of the origin to also move in the direction of the origin frame
        CONFIG.origin_R_ = RosTools::rotationMatrixFromHeading(-angle);

        for (auto &ped : pedestrians_) // Shift the peds start and goal to the origin
        {
            ped->start_.UndoTransform(origin_);
            ped->start_.Transform(msg.poses[0].pose, angle);

            ped->goal_.UndoTransform(origin_);
            ped->goal_.Transform(msg.poses[0].pose, angle);

            ped->Reset();
        }
        origin_ = msg.poses[0].pose;
    }
}

void PedestrianSimulator::SettingNCallback(const std_msgs::Float64 &msg)
{
    if (CONFIG.debug_output_)
        ROS_WARN_STREAM("Pedestrian Simulator: Received N = " << msg.data);

    CONFIG.horizon_N_ = msg.data;
}

void PedestrianSimulator::SettingdtCallback(const std_msgs::Float64 &msg)
{
    if (CONFIG.debug_output_)
        ROS_WARN_STREAM("Pedestrian Simulator: Received dt = " << msg.data);

    CONFIG.prediction_step_ = msg.data;
}

void PedestrianSimulator::SettingHzCallback(const std_msgs::Float64 &msg)
{
    if (CONFIG.debug_output_)
        ROS_WARN_STREAM("Pedestrian Simulator: Received update rate = " << msg.data);

    CONFIG.update_frequency_ = msg.data;
    CONFIG.delta_t_ = 1.0 / CONFIG.update_frequency_;
    timer_ = nh_.createTimer(ros::Duration(1.0 / (double)CONFIG.update_frequency_), &PedestrianSimulator::Poll, this); // Restart the timer
}

void PedestrianSimulator::Reset()
{
    vehicle_frame_ = geometry_msgs::Pose();

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

void PedestrianSimulator::Poll(const ros::TimerEvent &event)
{
    // Moving pedsim agents
    if (pedsim_manager_)
        pedsim_manager_->Update(CONFIG.delta_t_);

    if (CONFIG.debug_output_)
        ROS_INFO("PedestrianSimulator: Update");

    // std::cout << "preupdate\n";
    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
        ped->PreUpdateComputations();

    // std::cout << "update\n";
    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
    {
        ped->Update();
        // ped->MoveFrame(vehicle_speed_);
    }
    // std::cout << "done\n";

    Publish();
    PublishPredictions();

    PublishDebugVisuals();

    VisualizeRobot();

    VisualizePedestrians();

    VisualizeStaticObstacles();
}

void PedestrianSimulator::Publish()
{
    derived_object_msgs::ObjectArray ped_array_msg;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        derived_object_msgs::Object ped_msg;
        ped_msg.id = id;

        ped_msg.pose.position.x = ped->position_.x - vehicle_frame_.position.x;
        ped_msg.pose.position.y = ped->position_.y - vehicle_frame_.position.y;
        ped_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));

        ped_msg.twist = ped->noisy_twist_;

        // For the Carla pedestrian visual updates, we need to publish the positions and velocities here
        carla_position_pub_[id].publish(ped_msg.pose);
        // ped_msg.twist.linear.x *= CONFIG.delta_t_; // Account for the time step
        // ped_msg.twist.linear.y *= CONFIG.delta_t_; // Account for the time step
        carla_velocity_pub_[id].publish(ped_msg.twist);
        // ped_msg.shape.dimensions[0] = 0.25;
        // ped_msg.shape.dimensions[1] = 0.25;
        ped_array_msg.objects.push_back(ped_msg);
        id++;
    }

    ped_array_msg.header.stamp = ros::Time::now();
    ped_array_msg.header.frame_id = "map";

    obstacle_pub_.publish(ped_array_msg);

    if (CONFIG.pretend_to_be_optitrack_)
        PublishOptitrackPedestrians(ped_array_msg);
}

void PedestrianSimulator::PublishOptitrackPedestrians(const derived_object_msgs::ObjectArray &ped_msg)
{
    geometry_msgs::PoseStamped optitrack_msg;
    optitrack_msg.header.stamp = ped_msg.header.stamp;
    optitrack_msg.header.frame_id = ped_msg.header.frame_id;
    for (size_t ped_id = 0; ped_id < ped_msg.objects.size(); ped_id++)
    {
        optitrack_msg.pose = ped_msg.objects[ped_id].pose;    // Update the pose
        optitrack_publishers_[ped_id].publish(optitrack_msg); // Send the message over this publisher
    }
}

void PedestrianSimulator::PublishPredictions()
{
    // Decide how to compute predictions
    switch (CONFIG.ped_type_)
    {
    case PedestrianType::GAUSSIAN:

        PublishGaussianPredictions();
        return;
    case PedestrianType::BINOMIAL:
        PublishBinomialPredictions();
        return;
    case PedestrianType::SOCIAL:
        PublishSocialPredictions(); // With libpedsim
        return;
    default:
        if (CONFIG.constant_velocity_predictions_)
        {
            PublishGaussianPredictions();
            return;
        }
        break;
    }

    // Otherwise we use this function

    mpc_msgs::obstacle_array prediction_array;
    prediction_array.header.frame_id = "map";

    // Idea: Copy the pedestrian and step all of them one by one recording their positions then
    std::vector<std::unique_ptr<Pedestrian>> copied_pedestrians;
    std::vector<mpc_msgs::gaussian> gaussian_msgs;

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

        mpc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        // Initial position
        gmm_msg.pose.position.x = copied_ped->position_.x - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = copied_ped->position_.y - vehicle_frame_.position.y;
        gmm_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(copied_ped->twist_.linear.y, copied_ped->twist_.linear.x));

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
            ped->Update(CONFIG.prediction_step_); /** @note Using the prediction time step, not update step */

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = ped->position_.x;
            pose.pose.position.y = ped->position_.y;
            pose.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));

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
    obstacle_trajectory_prediction_pub_.publish(prediction_array);
}

void PedestrianSimulator::PublishSocialPredictions()
{

    mpc_msgs::obstacle_array prediction_array;
    prediction_array.header.frame_id = "map";

    // Build up the scene
    pedsim_prediction_manager_->Reset();

    for (auto &pedestrian : pedestrians_)
    {
        Ped::Tagent *a = pedsim_prediction_manager_->AddAgent(pedestrian->GetPosition()(0),
                                                              pedestrian->GetPosition()(1),
                                                              pedestrian->goal_.x,
                                                              pedestrian->goal_.y);

        a->setVelocity(pedestrian->GetSpeed()(0), pedestrian->GetSpeed()(1), 0.);
        a->setVmax(pedestrian->velocity_);
    }

    // Idea: Copy the pedestrian and step all of them one by one recording their positions then
    // std::vector<std::unique_ptr<Pedestrian>> copied_pedestrians;
    std::vector<mpc_msgs::gaussian> gaussian_msgs;

    // RobotState copied_robot = robot_state_;

    size_t id = 0;
    for (auto &ped : pedsim_prediction_manager_->GetAllAgents())
    {
        if (ped->gettype() == (int)AgentType::ROBOT)
            continue;

        mpc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        // Initial position
        gmm_msg.pose.position.x = ped->getx() - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = ped->gety() - vehicle_frame_.position.y;
        gmm_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->getvy(), ped->getvx()));

        prediction_array.obstacles.push_back(gmm_msg);

        gaussian_msgs.emplace_back();
        id++;
    }

    for (int k = 0; k < CONFIG.horizon_N_; k++)
    {
        // copied_robot.pos += copied_robot.vel * CONFIG.delta_t_; // Update the robot position assuming constant velocity
        pedsim_prediction_manager_->Update(CONFIG.prediction_step_);
        id = 0;
        for (auto &ped : pedsim_prediction_manager_->GetAllAgents())
        {
            if (ped->gettype() == (int)AgentType::ROBOT)
                continue;

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = ped->getx();
            pose.pose.position.y = ped->gety();
            pose.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->getvy(), ped->getvx()));

            gaussian_msgs[id].mean.poses.push_back(pose);

            // The variance is simply the uncertainty per stage
            gaussian_msgs[id].major_semiaxis.push_back(CONFIG.process_noise_[0]);
            gaussian_msgs[id].minor_semiaxis.push_back(CONFIG.process_noise_[1]);

            id++;
        }
    }

    id = 0;
    for (auto &ped : pedsim_prediction_manager_->GetAllAgents())
    {
        if (ped->gettype() == (int)AgentType::ROBOT)
            continue;

        prediction_array.obstacles[id].gaussians.push_back(gaussian_msgs[id]);
        prediction_array.obstacles[id].probabilities.push_back(1.0);
        id++;
    }
    prediction_array.header.stamp = ros::Time::now();
    obstacle_trajectory_prediction_pub_.publish(prediction_array);
}

void PedestrianSimulator::PublishGaussianPredictions()
{

    mpc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {

        mpc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = ped->position_.y - vehicle_frame_.position.y;
        gmm_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));

        // We simply load the uncertainty, to be integrated on the controller side
        mpc_msgs::gaussian gaussian_msg;
        geometry_msgs::PoseStamped pose;
        pose.pose = gmm_msg.pose;

        for (int k = 0; k < CONFIG.horizon_N_; k++) // 1 - N
        {
            Eigen::Vector2d rotated_predict = CONFIG.origin_R_ * Eigen::Vector2d(ped->twist_.linear.x, ped->twist_.linear.y);

            pose.pose.position.x += rotated_predict(0) * CONFIG.prediction_step_;
            pose.pose.position.y += rotated_predict(1) * CONFIG.prediction_step_;

            // We simply add the mean so that we can determine the samples in the controller
            gaussian_msg.mean.poses.push_back(pose);

            // The variance is simply the uncertainty per stage
            gaussian_msg.major_semiaxis.push_back(CONFIG.process_noise_[0]);
            gaussian_msg.minor_semiaxis.push_back(CONFIG.process_noise_[1]);
        }

        gmm_msg.gaussians.push_back(gaussian_msg);
        gmm_msg.probabilities.push_back(1.0);

        prediction_array.obstacles.push_back(gmm_msg);
        id++;
    }

    prediction_array.header.stamp = ros::Time::now();
    prediction_array.header.frame_id = "map";

    obstacle_trajectory_prediction_pub_.publish(prediction_array);
}

void PedestrianSimulator::PublishBinomialPredictions()
{
    mpc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &general_ped : pedestrians_)
    {
        BinomialPedestrian *ped = (BinomialPedestrian *)(general_ped.get());
        mpc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = ped->position_.y - vehicle_frame_.position.y;
        gmm_msg.pose.orientation = RosTools::angleToQuaternion(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));

        if (ped->state == PedState::STRAIGHT)
        {

            // For each state in the horizon we need to define a Gaussian + the case where the ped does not cross
            for (int k_mode = 0; k_mode < CONFIG.horizon_N_ + 1; k_mode++) // Mode k_mode is the mode where the ped switches to crossing at k=k_mode
            {
                mpc_msgs::gaussian gaussian_msg;
                // We simply load the uncertainty, to be integrated on the controller side
                geometry_msgs::PoseStamped pose;
                pose.pose = gmm_msg.pose;

                double prob = 1.;

                // Loop over each time step of motion prediction
                for (int k = 0; k < CONFIG.horizon_N_; k++)
                {
                    if ((k_mode == CONFIG.horizon_N_) || (k < k_mode)) // If this is the last mode OR we are not crossing yet
                    {
                        prob *= (1.0 - ped->p); // We did not start crossing yet
                        Eigen::Vector2d rotated_predict = CONFIG.origin_R_ * Eigen::Vector2d(ped->B_straight(0) * ped->direction_ * CONFIG.ped_velocity_ * CONFIG.prediction_step_, ped->B_straight(1) * ped->direction_ * CONFIG.ped_velocity_ * CONFIG.prediction_step_);

                        pose.pose.position.x += rotated_predict(0);
                        pose.pose.position.y += rotated_predict(1);
                    }
                    else // If we are crossing
                    {
                        if (k_mode == k)
                            prob *= ped->p; // We cross from here

                        Eigen::Vector2d rotated_predict = CONFIG.origin_R_ * Eigen::Vector2d(ped->B_cross(0) * ped->direction_ * CONFIG.ped_velocity_ * CONFIG.prediction_step_, ped->B_cross(1) * ped->direction_ * CONFIG.ped_velocity_ * CONFIG.prediction_step_);

                        pose.pose.position.x += rotated_predict(0);
                        pose.pose.position.y += rotated_predict(1);
                    }

                    // We simply add the mean so that we can determine the samples in the controller
                    gaussian_msg.mean.poses.push_back(pose);

                    // The variance is simply the uncertainty per stage
                    gaussian_msg.major_semiaxis.push_back(CONFIG.process_noise_[0]);
                    gaussian_msg.minor_semiaxis.push_back(CONFIG.process_noise_[1]);
                }

                gmm_msg.gaussians.push_back(gaussian_msg);
                gmm_msg.probabilities.push_back(prob);
            }
        }
        else // If we already crossed, move straight
        {
            mpc_msgs::gaussian gaussian_msg;
            geometry_msgs::PoseStamped pose;
            pose.pose = gmm_msg.pose;

            for (int k = 0; k < CONFIG.horizon_N_; k++)
            {
                Eigen::Vector2d rotated_predict = CONFIG.origin_R_ * Eigen::Vector2d(ped->B_cross(0) * ped->direction_ * CONFIG.ped_velocity_, ped->B_cross(1) * ped->direction_ * CONFIG.ped_velocity_);

                pose.pose.position.x += rotated_predict(0) * CONFIG.prediction_step_;
                pose.pose.position.y += rotated_predict(1) * CONFIG.prediction_step_;

                // We simply add the mean so that we can determine the samples in the controller
                gaussian_msg.mean.poses.push_back(pose);

                // The variance is simply the uncertainty per stage
                gaussian_msg.major_semiaxis.push_back(CONFIG.process_noise_[0]);
                gaussian_msg.minor_semiaxis.push_back(CONFIG.process_noise_[1]);
            }

            gmm_msg.gaussians.push_back(gaussian_msg);
            gmm_msg.probabilities.push_back(1.0);
        }

        prediction_array.obstacles.push_back(gmm_msg);
        id++;
    }
    prediction_array.header.stamp = ros::Time::now();
    prediction_array.header.frame_id = "map";

    obstacle_trajectory_prediction_pub_.publish(prediction_array);
}

void PedestrianSimulator::PublishDebugVisuals()
{
    RosTools::ROSPointMarker &arrow = debug_visuals_->getNewPointMarker("ARROW");
    arrow.setScale(0.8 * 1.5, 0.15 * 1.5, 0.15 * 1.5);

    RosTools::ROSPointMarker &start_goal_marker = debug_visuals_->getNewPointMarker("SPHERE"); // Plot the start and goal locations of the pedestrians
    start_goal_marker.setScale(0.4, 0.4, 0.4);

    RosTools::ROSPointMarker &robot_position = debug_visuals_->getNewPointMarker("Cube");
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

        arrow.setOrientation(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));
        geometry_msgs::Point point;
        point.x = ped->position_.x - vehicle_frame_.position.x;
        point.y = ped->position_.y - vehicle_frame_.position.y;
        point.z = 39.; // 1.9 / 2.0;
        arrow.addPointMarker(point);

        start_goal_marker.setColor(1., 0., 0.);
        start_goal_marker.addPointMarker(Eigen::Vector3d(ped->start_.x, ped->start_.y, 0.));

        start_goal_marker.setColor(0., 1., 0.);
        start_goal_marker.addPointMarker(Eigen::Vector3d(ped->goal_.x, ped->goal_.y, 0.));
    }

    for (auto &ped : pedestrians_)
    {
        // Only plot this if we are using the waypoint strategy
        if (dynamic_cast<WaypointPedestrian *>(ped.get()) == nullptr)
            continue;

        WaypointPedestrian fake_ped = *((WaypointPedestrian *)ped.get());

        for (size_t path_id = 0; path_id < fake_ped.paths_.size(); path_id++)
        {
            arrow.setColorInt(8);

            fake_ped.current_path_id_ = path_id; // Explicitly set the path ID

            bool found_waypoint = fake_ped.FindClosestWaypoint(); // Set the current waypoint via a search
            if (found_waypoint)
            {
                arrow.setColorInt(8);

                arrow.setOrientation(fake_ped.position_.Angle(fake_ped.GetCurrentWaypoint()));
                geometry_msgs::Point point;
                point.x = fake_ped.position_.x;
                point.y = fake_ped.position_.y;
                arrow.addPointMarker(point);
            }
        }
    }

    robot_position.addPointMarker(Eigen::Vector3d(robot_state_.pos(0), robot_state_.pos(1), 0.));

    debug_visuals_->publish();
}

void PedestrianSimulator::VisualizeRobot()
{
    if ((!pedsim_manager_) || pedsim_manager_->GetRobot() == nullptr)
        return;

    RosTools::ROSPointMarker &obstacle_marker = robot_visual_->getNewPointMarker("CYLINDER");
    obstacle_marker.setColorInt(0, 1.);
    obstacle_marker.setScale(pedsim_manager_->GetRobot()->getradius() * 2,
                             pedsim_manager_->GetRobot()->getradius() * 2,
                             0.1);

    obstacle_marker.addPointMarker(Eigen::Vector3d(pedsim_manager_->GetRobot()->getx(),
                                                   pedsim_manager_->GetRobot()->gety(), 0.05));

    robot_visual_->publish();
}

void PedestrianSimulator::VisualizePedestrians()
{

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    int cur_id = 0;
    for (auto &ped : pedestrians_) // Publish a pedestrian model
    {
        // marker.ns = "myns";
        // marker.id = k + current_cone_pos.left.x.size() + current_cone_pos.right.x.size() + 1;
        marker.id = ped->id_;

        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        // marker.action = visualization_msgs::Marker::ADD;
        marker.mesh_resource = "package://pedestrian_simulator/models/walking.dae";
        marker.pose.position.x = ped->position_.x;
        marker.pose.position.y = ped->position_.y;
        marker.pose.position.z = 0;

        // Account for the origin rotation in the velocity
        Eigen::Vector2d rotated_twist = CONFIG.origin_R_ * Eigen::Vector2d(ped->twist_.linear.x, ped->twist_.linear.y);
        double angle = std::atan2(rotated_twist(1), rotated_twist(0));
        tf::Quaternion q = tf::createQuaternionFromRPY(0., 0., angle + M_PI / 2.);
        geometry_msgs::Quaternion result;
        marker.pose.orientation.x = q.getX();
        marker.pose.orientation.y = q.getY();
        marker.pose.orientation.z = q.getZ();
        marker.pose.orientation.w = q.getW();

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Consistent with LMPCC
        int select = ped->id_;
        select %= (int)(colors_.size() / 3); // We only have 20 values
        select = (int)(colors_.size() / 3) - 1 - select;

        marker.color.r = colors_[3 * select + 0];
        marker.color.g = colors_[3 * select + 1];
        marker.color.b = colors_[3 * select + 2];

        marker.color.a = 1.0;
        // marker.lifetime = ros::Duration();
        markers.markers.push_back(marker);
        cur_id++;
    }
    ped_model_visuals_.publish(markers);
}

void PedestrianSimulator::VisualizeStaticObstacles()
{
    RosTools::ROSPointMarker &obstacle_marker = static_obstacle_visuals_->getNewPointMarker("Cube");
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

    static_obstacle_visuals_->publish();
}