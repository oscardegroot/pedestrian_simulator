#include "pedestrian_simulator/pedestrian_simulator.h"

PedestrianSimulator::PedestrianSimulator()
{
    ROS_INFO("PedestrianSimulator: Initializing");

    srand(time(NULL));

    Config::Get().Init();

    debug_visuals_.reset(new ROSMarkerPublisher(nh_, "pedestrian_simulator/debug", "map", 50)); //3500)); // was 1800

    obstacle_pub_ = nh_.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    obstacle_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/predictions", 1);
    obstacle_trajectory_prediction_pub_ = nh_.advertise<lmpcc_msgs::obstacle_array>("/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, &PedestrianSimulator::ResetCallback, this);

    xml_reader_.reset(new XMLReader());

    // Read pedestrian data
    switch (CONFIG.ped_type_)
    {
    case PedestrianType::WAYPOINT:
        xml_reader_->GetPedestrians(pedestrians_); //pedestrians_;
        break;
    case PedestrianType::GAUSSIAN:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new GaussianPedestrian(xml_reader_->pedestrians_[ped_id].start_, xml_reader_->pedestrians_[ped_id].paths_[0].back()));
        }
        break;
    case PedestrianType::BINOMIAL:
        for (size_t ped_id = 0; ped_id < xml_reader_->pedestrians_.size(); ped_id++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new BinomialPedestrian(xml_reader_->pedestrians_[ped_id].start_));
        }
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

void PedestrianSimulator::Reset()
{

    for (auto &ped : pedestrians_)
    {
        // Reset pedestrians to their starting position
        ped->Reset();
    }
}

void PedestrianSimulator::Poll(const ros::TimerEvent &event)
{
    ROS_INFO("PedestrianSimulator: Update");

    for (std::unique_ptr<Pedestrian> &ped : pedestrians_)
    {
        ped->Update();
    }

    Publish();
    PublishPredictions();
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

        ped_msg.pose.position.x = ped->position_.x;
        ped_msg.pose.position.y = ped->position_.y;

        ped_msg.twist = ped->twist_;

        // ped_msg.shape.dimensions[0] = 0.25;
        // ped_msg.shape.dimensions[1] = 0.25;
        ped_array_msg.objects.push_back(ped_msg);
        id++;
    }

    ped_array_msg.header.stamp = ros::Time::now();
    ped_array_msg.header.frame_id = "map";

    obstacle_pub_.publish(ped_array_msg);
}

void PedestrianSimulator::PublishPredictions()
{
    lmpcc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        lmpcc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x;
        gmm_msg.pose.position.y = ped->position_.y;

        ped->PredictPath(gmm_msg);

        prediction_array.obstacles.push_back(gmm_msg);
        id++;
    }

    prediction_array.header.stamp = ros::Time::now();
    prediction_array.header.frame_id = "map";

    obstacle_prediction_pub_.publish(prediction_array);
}

void PedestrianSimulator::PublishTrajectoryPredictions()
{
    lmpcc_msgs::obstacle_array prediction_array;

    unsigned int id = 0;
    for (auto &ped : pedestrians_)
    {
        lmpcc_msgs::obstacle_gmm gmm_msg;
        gmm_msg.id = id;

        gmm_msg.pose.position.x = ped->position_.x;
        gmm_msg.pose.position.y = ped->position_.y;

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
    arrow.setScale(1.0, 0.2, 0.2);

    for (auto &ped : pedestrians_)
    {
        arrow.setColor(1.0, 0.0, 0.0, 0.8);

        arrow.setOrientation(std::atan2(ped->twist_.linear.y, ped->twist_.linear.x));
        geometry_msgs::Point point;
        point.x = ped->position_.x;
        point.y = ped->position_.y;
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
            arrow.setColor(0.0, 1.0, 0.0, 1.0);

            fake_ped.current_path_id_ = path_id; // Explicitly set the path ID

            bool found_waypoint = fake_ped.FindClosestWaypoint(); // Set the current waypoint via a search
            if (found_waypoint)
            {
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