
#include <pedestrian_simulator/ros1_pedestrian_simulator.h>

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/configuration.h>

#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>
#include <ros_tools/logging.h>

#include <memory>

ROSPedestrianSimulator::ROSPedestrianSimulator()
{
    InitializePublishersAndSubscribers();

    _simulator = std::make_unique<PedestrianSimulator>();

    VISUALS.init(&_nh);
}

void ROSPedestrianSimulator::InitializePublishersAndSubscribers()
{
    ped_model_visuals_ = _nh.advertise<visualization_msgs::MarkerArray>("/pedestrian_simulator/visualization", 5);

    obstacle_pub_ = _nh.advertise<derived_object_msgs::ObjectArray>("/pedestrian_simulator/pedestrians", 1);
    obstacle_trajectory_prediction_pub_ = _nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 1);

    reset_sub_ = _nh.subscribe("/lmpcc/reset_environment", 1, &ROSPedestrianSimulator::ResetCallback, this);
    // vehicle_speed_sub_ = _nh.subscribe("/lmpcc/vehicle_speed", 1, &PedestrianSimulator::VehicleVelocityCallback, this);
    robot_state_sub_ = _nh.subscribe("/pedestrian_simulator/robot_state", 1, &ROSPedestrianSimulator::RobotStateCallback, this);

    setting_N_sub_ = _nh.subscribe("/pedestrian_simulator/horizon", 1, &ROSPedestrianSimulator::SettingNCallback, this);
    setting_dt_sub_ = _nh.subscribe("/pedestrian_simulator/integrator_step", 1, &ROSPedestrianSimulator::SettingdtCallback, this);
    setting_hz_sub_ = _nh.subscribe("/pedestrian_simulator/clock_frequency", 1, &ROSPedestrianSimulator::SettingHzCallback, this);

    if (CONFIG.use_path_origin_)
        path_origin_sub_ = _nh.subscribe("/roadmap/reference", 1, &ROSPedestrianSimulator::OriginCallback, this);

    start_server = _nh.advertiseService("/pedestrian_simulator/start", &ROSPedestrianSimulator::startCallback, this);
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
    timer_ = _nh.createTimer(ros::Duration(1.0 / CONFIG.update_frequency_), &ROSPedestrianSimulator::Loop, this);
    return true;
}

void ROSPedestrianSimulator::ResetCallback(const std_msgs::Empty &msg)
{
    // LOG_INFO("PedestrianSimulator: Reset callback");
    _simulator->Reset();
}

void ROSPedestrianSimulator::RobotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _simulator->SetRobotState(RobotState(
        Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y),
        RosTools::quaternionToAngle(msg->pose.orientation),
        0.));
}

void ROSPedestrianSimulator::VehicleVelocityCallback(const geometry_msgs::Twist &msg)
{

    // _simulator->vehicle_frame_(0) += msg.linear.x * CONFIG.delta_t_;
    // _simulator->vehicle_frame_(1) += msg.linear.y * CONFIG.delta_t_;
}

void ROSPedestrianSimulator::OriginCallback(const nav_msgs::Path &msg)
{
    // double angle = std::atan2(msg.poses[1].pose.position.y - msg.poses[0].pose.position.y, msg.poses[1].pose.position.x - msg.poses[0].pose.position.x);

    // // Update if the path is new
    // if (origin_.position.x != msg.poses[0].pose.position.x || origin_.position.y != msg.poses[0].pose.position.y || RosTools::rotationMatrixFromHeading(-angle) != CONFIG.origin_R_)
    // {
    //     // We save the rotation of the origin to also move in the direction of the origin frame
    //     CONFIG.origin_R_ = RosTools::rotationMatrixFromHeading(-angle);

    //     for (auto &ped : pedestrians_) // Shift the peds start and goal to the origin
    //     {
    //         ped->start_.UndoTransform(origin_);
    //         ped->start_.Transform(msg.poses[0].pose, angle);

    //         ped->goal_.UndoTransform(origin_);
    //         ped->goal_.Transform(msg.poses[0].pose, angle);

    //         ped->Reset();
    //     }
    //     origin_ = msg.poses[0].pose;
    // }
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
    _simulator->Loop();

    // std::vector<Prediction> obstacles = _simulator->GetPedestrians();
    // derived_object_msgs::ObjectArray obstacles_msg = PredictionsToObjectArray(obstacles);
    // obstacle_pub_.publish(obstacles_msg);

    // std::vector<Prediction> predictions = _simulator->GetPredictions();
    // mpc_planner_msgs::ObstacleArray predictions_msg = PredictionsToObstacleArray(predictions);
    // obstacle_trajectory_prediction_pub_.publish(predictions_msg);

    std::vector<Prediction> obstacles = _simulator->GetPedestrians();
    costmap_converter::ObstacleArrayMsg obstacles_msg = PredictionsToObjectArray(obstacles);
    obstacle_trajectory_prediction_pub_.publish(obstacles_msg);
}

// mpc_planner_msgs::ObstacleArray ROSPedestrianSimulator::PredictionsToObstacleArray(const std::vector<Prediction> &predictions)
// {
//     mpc_planner_msgs::ObstacleArray obstacle_array_msg;

//     for (auto &prediction : predictions)
//     {
//         obstacle_array_msg.obstacles.emplace_back();
//         obstacle_array_msg.obstacles.back().id = prediction.id;
//         obstacle_array_msg.obstacles.back().gaussians.emplace_back();
//         obstacle_array_msg.obstacles.back().pose.position.x = prediction.pos[0](0);
//         obstacle_array_msg.obstacles.back().pose.position.y = prediction.pos[0](1);
//         obstacle_array_msg.obstacles.back().pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);

//         auto &gaussian = obstacle_array_msg.obstacles.back().gaussians.back();

//         for (size_t i = 1; i < prediction.pos.size(); i++) // The first is the current position
//         {
//             gaussian.mean.poses.emplace_back();
//             gaussian.mean.poses.back().pose.position.x = prediction.pos[i](0);
//             gaussian.mean.poses.back().pose.position.y = prediction.pos[i](1);
//             gaussian.mean.poses.back().pose.position.z = 0.;
//             gaussian.mean.poses.back().pose.orientation = RosTools::angleToQuaternion(prediction.angle[i]);

//             gaussian.major_semiaxis.push_back(prediction.major_axis[i]);
//             gaussian.minor_semiaxis.push_back(prediction.minor_axis[i]);
//         }

//         obstacle_array_msg.obstacles.back().probabilities.push_back(1.);
//     }
//     obstacle_array_msg.header.stamp = ros::Time::now();
//     obstacle_array_msg.header.frame_id = "map";
//     return obstacle_array_msg;
// }

// derived_object_msgs::ObjectArray ROSPedestrianSimulator::PredictionsToObjectArray(const std::vector<Prediction> &predictions)
// {
//     derived_object_msgs::ObjectArray ped_array_msg;

//     for (auto &prediction : predictions)
//     {
//         derived_object_msgs::Object ped_msg;
//         ped_msg.id = prediction.id;
//         ped_msg.pose.position.x = prediction.pos[0](0);
//         ped_msg.pose.position.y = prediction.pos[0](1);
//         ped_msg.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);
//         ped_msg.twist.linear.x = prediction.vel[0](0);
//         ped_msg.twist.linear.y = prediction.vel[0](1);
//         ped_msg.twist.linear.z = 0.;

//         ped_array_msg.objects.push_back(ped_msg);
//     }
//     ped_array_msg.header.stamp = ros::Time::now();
//     ped_array_msg.header.frame_id = "map";

//     return ped_array_msg;
// }

costmap_converter::ObstacleArrayMsg ROSPedestrianSimulator::PredictionsToObjectArray(const std::vector<Prediction> &predictions)
{
    costmap_converter::ObstacleArrayMsg prediction_array;
    int id = 0;
    for (auto &prediction : predictions)
    {
        costmap_converter::ObstacleMsg obstacle_msg;

        obstacle_msg.id = id;
        obstacle_msg.radius = 0.4;

        // Initial position
        geometry_msgs::Polygon polygon;
        polygon.points.emplace_back();
        auto &point = polygon.points[0];
        point.x = prediction.pos[0](0);
        point.y = prediction.pos[0](1);
        obstacle_msg.polygon = polygon;

        obstacle_msg.orientation = RosTools::angleToQuaternion(prediction.angle[0]);
        obstacle_msg.velocities.twist.linear.x = prediction.vel[0](0);
        obstacle_msg.velocities.twist.linear.y = prediction.vel[0](1);

        prediction_array.obstacles.push_back(obstacle_msg);
        id++;
    }

    // CORRIDOR ONLY
    std::vector<double> first_x = {-2, -2, -2, 28};
    std::vector<double> first_y = {-2, -2, 28, -2};
    std::vector<double> second_x = {28, -2, 28, 28};
    std::vector<double> second_y = {-2, 28, 28, 28};
    for (size_t i = 0; i < first_x.size(); i++)
    {
        costmap_converter::ObstacleMsg obstacle_msg;
        obstacle_msg.id = id;

        // Initial position
        geometry_msgs::Polygon polygon;

        /*
         <obstacle x1="-2" y1="-2" x2="28" y2="-2"/>
        <obstacle x1="-2" y1="-2" x2="-2" y2="28"/>
        <obstacle x1="-2" y1="28" x2="28" y2="28"/>
        <obstacle x1="28" y1="-2" x2="28" y2="28"/>
         */
        polygon.points.emplace_back();
        polygon.points.emplace_back();
        auto &point_one = polygon.points[0];
        point_one.x = first_x[i];
        point_one.y = first_y[i];

        auto &point_two = polygon.points[1];
        point_two.x = second_x[i];
        point_two.y = second_y[i];
        obstacle_msg.polygon = polygon;

        obstacle_msg.orientation.x = 0.;
        obstacle_msg.orientation.y = 0.;
        obstacle_msg.orientation.z = 0.;
        obstacle_msg.orientation.w = 1.;
        obstacle_msg.velocities.twist.linear.x = 0;
        obstacle_msg.velocities.twist.linear.y = 0;

        prediction_array.obstacles.push_back(obstacle_msg);
    }
    prediction_array.header.stamp = ros::Time::now();
    prediction_array.header.frame_id = "map";

    return prediction_array;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ROSPedestrianSimulator sim;

    ros::spin();

    return 0;
}
