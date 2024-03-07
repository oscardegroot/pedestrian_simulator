
#include <pedestrian_simulator/ros1_pedestrian_simulator.h>
#include <pedestrian_simulator/pedestrian_simulator.h>

#include <ros/ros.h>

#include <ros_tools/visuals.h>

#include <memory>

std::unique_ptr<PedestrianSimulator> simulator;

void ROSPedestrianSimulator::initializePublishersAndSubscribers(ros::NodeHandle &nh)
{
    ped_model_visuals_ = nh.advertise<visualization_msgs::MarkerArray>("/pedestrian_simulator/visualization", 5);

    obstacle_pub_ = nh.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    // obstacle_prediction_pub_ = nh_.advertise<mpc_planner_msgs::obstacle_array>("/pedestrian_simulator/predictions", 1);
    obstacle_trajectory_prediction_pub_ = nh.advertise<mpc_planner_msgs::obstacle_array>("/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = nh.subscribe("/lmpcc/reset_environment", 1, &PedestrianSimulator::ResetCallback, simulator);
    // vehicle_speed_sub_ = nh.subscribe("/lmpcc/vehicle_speed", 1, &PedestrianSimulator::VehicleVelocityCallback, this);
    robot_state_sub_ = nh.subscribe("/robot_state", 1, &PedestrianSimulator::RobotStateCallback, simulator);

    setting_N_sub_ = nh.subscribe("/pedestrian_simulator/horizon", 1, &PedestrianSimulator::SettingNCallback, simulator);
    setting_dt_sub_ = nh.subscribe("/pedestrian_simulator/integrator_step", 1, &PedestrianSimulator::SettingdtCallback, simulator);
    setting_hz_sub_ = nh.subscribe("/pedestrian_simulator/clock_frequency", 1, &PedestrianSimulator::SettingHzCallback, simulator);

    for (size_t ped_id = 0; ped_id < pedestrians_.size(); ped_id++)
    {
        if (CONFIG.pretend_to_be_optitrack_)
            optitrack_publishers_.push_back(nh_.advertise<geometry_msgs::PoseStamped>("mocap_obstacle" + std::to_string(ped_id + 1) + "/pose", 1));
    }

    // The robot state publisher
    if (CONFIG.pretend_to_be_optitrack_)
    {
        optitrack_publishers_.push_back(nh_.advertise<geometry_msgs::PoseStamped>("Robot_1/pose", 1));
        joystick_publisher_ = nh_.advertise<sensor_msgs::Joy>("/bluetooth_teleop/joy", 1); // Disable the deadman switch
    }

    if (CONFIG.use_path_origin_)
        path_origin_sub_ = nh_.subscribe("/roadmap/reference", 1, &PedestrianSimulator::OriginCallback, this);

    start_server = nh_.advertiseService("/pedestrian_simulator/start", &PedestrianSimulator::startCallback, this);
}

bool ROSPedestrianSimulator::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (CONFIG.debug_output_)
        LOG_WARN("Pedestrian Simulator: Start service called.");

    if (!set_N_)
    {
        LOG_INFO("Pedestrian Simulator: Horizon not set yet, ignoring start call");
        return false;
    }

    if (!set_dt_)
    {
        LOG_INFO("Pedestrian Simulator: prediction step not set yet, ignoring start call");
        return false;
    }

    if (!set_hz_)
    {
        LOG_INFO("Pedestrian Simulator: Update frequency not set yet, ignoring start call");
        return false;
    }

    LOG_INFO("Pedestrian Simulator [horizon = " << CONFIG.horizon_N_ << ", dt = " << CONFIG.prediction_step_ << ", Hz = " << CONFIG.update_frequency_ << "]");

    // Implement the service server logic here
    timer_ = nh_.createTimer(ros::Duration(1.0 / CONFIG.update_frequency_), &PedestrianSimulator::Poll, this);
    return true;
}

void ROSPedestrianSimulator::ResetCallback(const std_msgs::Empty &msg)
{
    // LOG_INFO("PedestrianSimulator: Reset callback");
    Reset();
}

void ROSPedestrianSimulator::RobotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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

void ROSPedestrianSimulator::VehicleVelocityCallback(const geometry_msgs::Twist &msg)
{

    vehicle_frame_.position.x += msg.linear.x * CONFIG.delta_t_;
    vehicle_frame_.position.y += msg.linear.y * CONFIG.delta_t_;
    vehicle_frame_.orientation.z += msg.angular.z * CONFIG.delta_t_; // angular velocity is stored in orientation / angular .z (euler not quaternion)
}

void ROSPedestrianSimulator::OriginCallback(const nav_msgs::Path &msg)
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

void ROSPedestrianSimulator::SettingNCallback(const std_msgs::Int32 &msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: horizon = " << (int)msg.data);

    CONFIG.horizon_N_ = (int)msg.data;
    set_N_ = true;
}

void ROSPedestrianSimulator::SettingdtCallback(const std_msgs::Float32 &msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: integrator_step = " << (double)msg.data);

    CONFIG.prediction_step_ = (double)msg.data;
    set_dt_ = true;
}

void ROSPedestrianSimulator::SettingHzCallback(const std_msgs::Float32 &msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: clock_frequency = " << (double)msg.data);

    CONFIG.update_frequency_ = (double)msg.data;
    CONFIG.delta_t_ = 1.0 / CONFIG.update_frequency_;

    set_hz_ = true;
}

void ROSPedestrianSimulator::Loop(const ros::TimerEvent &event)
{
    simulator.Loop();

    mpc_planner_msgs::ObstacleArray predictions = PredictionsToObstacleArray(PublishPredictions());
    obstacle_trajectory_prediction_pub_.publish(predictions);
}

mpc_planner_msgs::ObstacleArray ROSPedestrianSimulator::PredictionsToObstacleArray(const std::vector<Prediction> &predictions)
{
    mpc_planner_msgs::ObstacleArray obstacle_array_msg;

    for (auto &prediction : predictions)
    {
        obstacle_array_msg.obstacles.emplace_back();
        obstacle_array_msg.obstacles.back().id = prediction.id;
        obstacle_array_msg.obstacles.gaussians.emplace_back();

        auto &gaussian = obstacle_array_msg.obstacles.gaussians.back();

        for (size_t i = 1; i < prediction.pos.size(); i++) // The first is the current position
        {
            gaussian.mean.poses.emplace_back();
            gaussian.mean.poses.back().position.x = prediction.pos[i](0);
            gaussian.mean.poses.back().position.y = prediction.pos[i](1);
            gaussian.mean.poses.back().position.z = 0.;
            gaussian.mean.poses.back().orientation = RosTools::angleToQuaternion(prediction.angle[i]);

            gaussian.major_axis.push_back(prediction.major_axis[i]);
            gaussian.minor_axis.push_back(prediction.minor_axis[i]);
        }

        obstacle_array_msg.obstacles.back().probabilities.push_back(1.);
    }
    obstacle_array_msg.header.stamp = ros::Time::now();
    obstacle_array_msg.header.frame_id = "map";
    return obstacle_array;
}

derived_object_msgs::ObjectArray ROSPedestrianSimulator::PredictionsToObjectArray(const std::vector<Prediction> &predictions)
{
    derived_object_msgs::ObjectArray ped_array_msg;

    for (auto &prediction : predictions)
    {
        derived_object_msgs::Object ped_msg;
        ped_msg.id = prediction.id;
        ped_msg.pose.position.x = prediction.pos[0](0);
        ped_msg.pose.position.y = prediction.pos[0](1);
        ped_msg.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);
        ped_msg.velocity.x = prediction.vel[0](0);
        ped_msg.velocity.y = prediction.vel[0](1);
        ped_msg.velocity.z = 0.;

        ped_array_msg.objects.push_back(ped_msg);
    }
    ped_array_msg.header.stamp = ros::Time::now();
    ped_array_msg.header.frame_id = "map";
}

void ROSPedestrianSimulator::Publish()
{

    obstacle_pub_.publish(PredictionsToObjectArray(ped_array_msg));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;

    initializePublishersAndSubscribers(&nh)
        VISUALS.init(&nh_);

    simulator.reset(new ROSPedestrianSimulator());

    ros::spin();

    return 0;
}
