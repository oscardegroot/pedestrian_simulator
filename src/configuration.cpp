
#include <pedestrian_simulator/configuration.h>

Config::~Config()
{
}

// read predicitve configuration paramter from paramter server
void Config::Init() // const std::string& node_handle_name
{
  ros::NodeHandle nh;

  retrieveParameter(nh, "pedestrian_simulator/node/debug_output", debug_output_, true);
  retrieveParameter(nh, "pedestrian_simulator/node/scenario", scenario_file_);
  retrieveParameter(nh, "pedestrian_simulator/node/update_frequency", update_frequency_, 20.);
  delta_t_ = 1.0 / update_frequency_;

  retrieveParameter(nh, "pedestrian_simulator/pedestrians/seed", seed_);
  retrieveParameter(nh, "pedestrian_simulator/pedestrians/process_noise", process_noise_);
  retrieveParameter(nh, "pedestrian_simulator/pedestrians/velocity", ped_velocity_);

  retrieveParameter(nh, "pedestrian_simulator/binomial/p", p_binomial_);

  retrieveParameter(nh, "pedestrian_simulator/static", static_, false);

  // Define the pedestrian type (string -> enum class)
  retrieveParameter(nh, "pedestrian_simulator/pedestrians/type", ped_type_string_);
  if (ped_type_string_.compare("waypoint") == 0)
    ped_type_ = PedestrianType::WAYPOINT;
  else if (ped_type_string_.compare("gaussian") == 0)
    ped_type_ = PedestrianType::GAUSSIAN;
  else if (ped_type_string_.compare("binomial") == 0)
    ped_type_ = PedestrianType::BINOMIAL;

  // retrieveParameter(nh, "pedestrian_simulator/external_waypoint_topic", external_waypoint_topic_, std::string());

  success_ = true;

  if (debug_output_)
    ROS_WARN("[Pedestrian Simulator]: Configuration initialized");
}
