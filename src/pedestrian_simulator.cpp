#include "pedestrian_simulator/pedestrian_simulator.h"

PedestrianSimulator::PedestrianSimulator()
{
    ROS_INFO("PedestrianSimulator: Initializing");

    Config::Get().Init();

    debug_visuals_.reset(new ROSMarkerPublisher(nh_, "pedestrian_simulator/debug", "map", 50)); // 3500)); // was 1800

    ped_model_visuals_ = nh_.advertise<visualization_msgs::MarkerArray>("/pedestrian_simulator/visualization", 5);

    obstacle_pub_ = nh_.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    // obstacle_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/predictions", 1);
    obstacle_trajectory_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, &PedestrianSimulator::ResetCallback, this);
    // vehicle_speed_sub_ = nh_.subscribe("/lmpcc/vehicle_speed", 1, &PedestrianSimulator::VehicleVelocityCallback, this);

    setting_N_sub_ = nh_.subscribe("/pedestrian_simulator/N", 1, &PedestrianSimulator::SettingNCallback, this);
    setting_dt_sub_ = nh_.subscribe("/pedestrian_simulator/dt", 1, &PedestrianSimulator::SettingdtCallback, this);
    // setting_hz_sub_ = nh_.subscribe("/pedestrian_simulator/Hz", 1, &PedestrianSimulator::SettingHzCallback, this);

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
            if (xml_reader_->is_random_[ped_id])
            {
                pedestrians_.back().reset(new RandomGaussianPedestrian(xml_reader_->random_x_min_[ped_id], xml_reader_->random_x_max_[ped_id],
                                                                       xml_reader_->random_y_min_[ped_id], xml_reader_->random_y_max_[ped_id],
                                                                       ped_id));
            }
            else
            {
                std::cout << ped_id << std::endl;
                pedestrians_.back().reset(new GaussianPedestrian(xml_reader_->pedestrians_[ped_id]->start_, xml_reader_->pedestrians_[ped_id]->goal_, ped_id));
            }
        }
        break;
    case PedestrianType::BINOMIAL:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new BinomialPedestrian(xml_reader_->pedestrians_[ped_id]->start_, ped_id));
        }
    }

    for (size_t i = 0; i < pedestrians_.size(); i++) // Assign IDs for all pedestrians
        pedestrians_[i]->id_ = i;

    // For Carla only!
    for (size_t ped_id = 0; ped_id < pedestrians_.size(); ped_id++)
    {
        carla_position_pub_.push_back(nh_.advertise<geometry_msgs::Pose>("/carla/simulated_peds/" + std::to_string(ped_id) + "/position", 1));
        carla_velocity_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("/carla/simulated_peds/" + std::to_string(ped_id) + "/velocity", 1));
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

void PedestrianSimulator::VehicleVelocityCallback(const geometry_msgs::Twist &msg)
{

    vehicle_frame_.position.x += msg.linear.x * CONFIG.delta_t_;
    vehicle_frame_.position.y += msg.linear.y * CONFIG.delta_t_;
    vehicle_frame_.orientation.z += msg.angular.z * CONFIG.delta_t_; // angular velocity is stored in orientation / angular .z (euler not quaternion)
}

void PedestrianSimulator::OriginCallback(const nav_msgs::Path &msg)
{

    // Update if the path is new
    if (origin_.position.x != msg.poses[0].pose.position.x && origin_.position.y != msg.poses[0].pose.position.y)
    {
        // We save the rotation of the origin to also move in the direction of the origin frame
        double angle = std::atan2(msg.poses[1].pose.position.y - msg.poses[0].pose.position.y, msg.poses[1].pose.position.x - msg.poses[0].pose.position.x);
        CONFIG.origin_R_ = Helpers::rotationMatrixFromHeading(-angle);

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
    ROS_WARN_STREAM("Pedestrian Simulator: Received N = " << msg.data);
    CONFIG.horizon_N_ = msg.data;
}

void PedestrianSimulator::SettingdtCallback(const std_msgs::Float64 &msg)
{
    ROS_WARN_STREAM("Pedestrian Simulator: Received dt = " << msg.data);
    CONFIG.prediction_step_ = msg.data;
}

void PedestrianSimulator::SettingHzCallback(const std_msgs::Float64 &msg)
{
    ROS_WARN_STREAM("Pedestrian Simulator: Received update rate = " << msg.data);
    CONFIG.update_frequency_ = msg.data;
    CONFIG.delta_t_ = 1.0 / CONFIG.update_frequency_;
    timer_ = nh_.createTimer(ros::Duration(1.0 / (double)CONFIG.update_frequency_), &PedestrianSimulator::Poll, this); // Restart the timer
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
    VisualizePedestrians();
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
            for (int k_mode = 0; k_mode < CONFIG.horizon_N_ + 1; k_mode++) // Mode k_mode is the mode where the ped switches to crossing at k=k_mode
            {
                lmpcc_msgs::gaussian gaussian_msg;
                // We simply load the uncertainty, to be integrated on the controller side
                geometry_msgs::PoseStamped pose;
                pose.pose = gmm_msg.pose;

                double prob = 1.;

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
            lmpcc_msgs::gaussian gaussian_msg;
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

void PedestrianSimulator::PublishDebugVisuals()
{
    ROSPointMarker &arrow = debug_visuals_->getNewPointMarker("ARROW");
    arrow.setScale(0.8 * 1.5, 0.15 * 1.5, 0.15 * 1.5);

    ROSPointMarker &start_goal_marker = debug_visuals_->getNewPointMarker("SPHERE"); // Plot the start and goal locations of the pedestrians
    start_goal_marker.setScale(0.4, 0.4, 0.4);

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
    debug_visuals_->publish();
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
        std::cout << ped->id_ << std::endl;
        marker.color.a = 1.0;
        // marker.lifetime = ros::Duration();
        markers.markers.push_back(marker);
        cur_id++;
    }
    ped_model_visuals_.publish(markers);
}
