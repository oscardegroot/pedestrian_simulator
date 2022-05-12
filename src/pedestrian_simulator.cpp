#include "pedestrian_simulator/pedestrian_simulator.h"

PedestrianSimulator::PedestrianSimulator()
{
    ROS_INFO("PedestrianSimulator: Initializing");

    Config::Get().Init();

    debug_visuals_.reset(new ROSMarkerPublisher(nh_, "pedestrian_simulator/debug", "map", 50)); // 3500)); // was 1800

    obstacle_pub_ = nh_.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    // obstacle_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/predictions", 1);
    obstacle_trajectory_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, &PedestrianSimulator::ResetCallback, this);
    vehicle_speed_sub_ = nh_.subscribe("/lmpcc/vehicle_speed", 1, &PedestrianSimulator::VehicleVelocityCallback, this);

    xml_reader_.reset(new XMLReader());

    // Read pedestrian data
    switch (CONFIG.ped_type_)
    {
    case PedestrianType::WAYPOINT:
        xml_reader_->GetPedestrians(pedestrians_); // pedestrians_;
        break;
    case PedestrianType::GAUSSIAN:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new GaussianPedestrian(xml_reader_->pedestrians_[ped_id].start_, xml_reader_->pedestrians_[ped_id].paths_[0].back(), ped_id));
        }
        break;
    case PedestrianType::BINOMIAL:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new BinomialPedestrian(xml_reader_->pedestrians_[ped_id].start_, ped_id));
        }
    }

    // For Carla only!
    for (size_t ped_id = 0; ped_id < pedestrians_.size(); ped_id++)
    {
        carla_position_pub_.push_back(nh_.advertise<geometry_msgs::Pose>("/carla/simulated_peds/" + std::to_string(ped_id) + "/position", 1));
        carla_velocity_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("/carla/simulated_peds/" + std::to_string(ped_id) + "/velocity", 1));
    }

    // Pick a path
    Reset();

    // Initialize the node loop
    timer_ = nh_.createTimer(ros::Duration(1.0 / CONFIG.update_frequency_), &PedestrianSimulator::Poll, this);

    ROS_INFO("PedestrianSimulator: Ready");
}

void PedestrianSimulator::ResetCallback(const std_msgs::Empty &msg)
{
    // ROS_INFO("PedestrianSimulator: Reset callback");
    Reset();
}

void PedestrianSimulator::VehicleVelocityCallback(const geometry_msgs::Twist &msg)
{
    vehicle_frame_.position.x += msg.linear.x * CONFIG.delta_t_;
    vehicle_frame_.position.y += msg.linear.y * CONFIG.delta_t_;
    vehicle_frame_.orientation.z += msg.angular.z * CONFIG.delta_t_; // angular velocity is stored in orientation / angular .z (euler not quaternion)
}

void PedestrianSimulator::Reset()
{
    vehicle_frame_ = geometry_msgs::Pose();

    for (auto &ped : pedestrians_)
    {
        // Reset pedestrians to their starting position
        ped->Reset();
    }
}

void PedestrianSimulator::Poll(const ros::TimerEvent &event)
{
    if (CONFIG.debug_output_)
        ROS_INFO("PedestrianSimulator: Update");

    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
    {
        ped->Update();
        // ped->MoveFrame(vehicle_speed_);
    }

    Publish();
    PublishTrajectoryPredictions();
    PublishDebugVisuals();
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
}

void PedestrianSimulator::PublishBinomialTrajectoryPredictions()
{
    lmpcc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &general_ped : pedestrians_)
    {
        BinomialPedestrian *ped = (BinomialPedestrian *)(general_ped.get());
        lmpcc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = ped->position_.y - vehicle_frame_.position.y;

        if (ped->state == PedState::STRAIGHT)
        {

            // For each state in the horizon we need to define a Gaussian + the case where the ped does not cross
            for (int k_mode = 0; k_mode < HORIZON_N + 1; k_mode++) // Mode k_mode is the mode where the ped switches to crossing at k=k_mode
            {
                lmpcc_msgs::gaussian gaussian_msg;
                // We simply load the uncertainty, to be integrated on the controller side
                geometry_msgs::PoseStamped pose;
                pose.pose = gmm_msg.pose;

                double prob = 1.;

                for (int k = 0; k < HORIZON_N; k++)
                {
                    if ((k_mode == HORIZON_N) || (k < k_mode)) // If this is the last mode OR we are not crossing yet
                    {
                        prob *= (1.0 - ped->p); // We did not start crossing yet
                        pose.pose.position.x += ped->B_straight(0) * ped->direction_ * CONFIG.ped_velocity_ /* std::cos(vehicle_frame_.orientation.z)*/ * DELTA_T_PREDICT;
                        pose.pose.position.y += ped->B_straight(1) * ped->direction_ * CONFIG.ped_velocity_ /* std::sin(vehicle_frame_.orientation.z)*/ * DELTA_T_PREDICT;
                    }
                    else // If we are crossing
                    {
                        if (k_mode == k)
                            prob *= ped->p; // We cross from here

                        pose.pose.position.x += ped->B_cross(0) * ped->direction_ * CONFIG.ped_velocity_ /* std::cos(vehicle_frame_.orientation.z)*/ * DELTA_T_PREDICT;
                        pose.pose.position.y += ped->B_cross(1) * ped->direction_ * CONFIG.ped_velocity_ /* std::sin(vehicle_frame_.orientation.z)*/ * DELTA_T_PREDICT;
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
            lmpcc_msgs::gaussian gaussian_msg;
            geometry_msgs::PoseStamped pose;
            pose.pose = gmm_msg.pose;

            for (int k = 0; k < HORIZON_N; k++)
            {
                pose.pose.position.x += ped->B_cross(0) * ped->direction_ * CONFIG.ped_velocity_ * DELTA_T_PREDICT;
                pose.pose.position.y += ped->B_cross(1) * ped->direction_ * CONFIG.ped_velocity_ * DELTA_T_PREDICT;

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

void PedestrianSimulator::PublishTrajectoryPredictions()
{
    if (CONFIG.ped_type_ == PedestrianType::BINOMIAL)
    {

        PublishBinomialTrajectoryPredictions();
        return;
    }

    lmpcc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        lmpcc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x - vehicle_frame_.position.x;
        gmm_msg.pose.position.y = ped->position_.y - vehicle_frame_.position.y;

        // We simply load the uncertainty, to be integrated on the controller side
        lmpcc_msgs::gaussian gaussian_msg;
        geometry_msgs::PoseStamped pose;
        pose.pose = gmm_msg.pose;

        for (int k = 0; k < HORIZON_N; k++)
        {
            pose.pose.position.x += ped->twist_.linear.x * DELTA_T_PREDICT;
            pose.pose.position.y += ped->twist_.linear.y * DELTA_T_PREDICT;

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

void PedestrianSimulator::PublishDebugVisuals()
{
    ROSPointMarker &arrow = debug_visuals_->getNewPointMarker("ARROW");
    arrow.setScale(0.8 * 1.5, 0.15 * 1.5, 0.15 * 1.5);

    for (auto &ped : pedestrians_)
    {
        arrow.setColor(1.0, 0.0, 0.0, 1.);

        // Eigen::Matrix3d H;
        // H.block(0, 0, 2, 2) = Helpers::rotationMatrixFromHeading(vehicle_frame_.orientation.z);
        // H.block(2, 0, 2, 1) = Eigen::Vector2d(vehicle_frame_.position.x, vehicle_frame_.position.y);
        // H(2, 2) = 1.0;
        // Eigen::Vector3d pr = H * Eigen::Vector3d(ped->position_.x, ped->position_.y, 1);
        // arrow.setColorInt(15);
        // arrow.setColor(247. / 256., 177. / 256., 64. / 256.);

        arrow.setOrientation(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));
        geometry_msgs::Point point;
        point.x = ped->position_.x - vehicle_frame_.position.x;
        point.y = ped->position_.y - vehicle_frame_.position.y;
        point.z = 1.9 / 2.0;
        arrow.addPointMarker(point);
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
    debug_visuals_->publish();
}