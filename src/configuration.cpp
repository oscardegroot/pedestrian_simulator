
#include <pedestrian_simulator/configuration.h>
namespace pedestrian_simulator
{
  Config::~Config()
  {
  }

  // read predicitve configuration paramter from paramter server
  void Config::Init(rclcpp::Node *node) // const std::string& node_handle_name
  {
    RCLCPP_INFO(node->get_logger(), "Initializing2");

    auto parameters = node->list_parameters({"/**"}, 3); // std::make_shared<rclcpp::parameter::ParameterClient>(node);
    RCLCPP_INFO(node->get_logger(), "Initializing3");
    // Print the parameter names and values
    for (const auto &parameter : parameters.names)
    {
      RCLCPP_INFO(node->get_logger(), "Parameter Name: %s", parameter.c_str());
      // RCLCPP_INFO(node->get_logger(), "Parameter Name: %s, Value: %s",
      // parameter.c_str(), node->get_parameter(parameter).as_string());
    }

    // node->declare_parameter("test", "test_value");
    retrieveParameter(node, "pedestrian_simulator.node.debug_output", debug_output_, true);
    retrieveParameter(node, "pedestrian_simulator.node.scenario", scenario_file_);
    retrieveParameter(node, "pedestrian_simulator.node.update_frequency", update_frequency_, 20.);
    delta_t_ = 1.0 / ((double)update_frequency_);

    retrieveParameter(node, "pedestrian_simulator.node.pretend_to_be_optitrack", pretend_to_be_optitrack_, false);

    retrieveParameter(node, "pedestrian_simulator.node.prediction_step", prediction_step_, delta_t_);
    retrieveParameter(node, "pedestrian_simulator.node.horizon", horizon_N_);
    retrieveParameter(node, "pedestrian_simulator.node.use_path_origin", use_path_origin_);

    retrieveParameter(node, "obstacles.radius", ped_radius_, 0.5);

    retrieveParameter(node, "pedestrian_simulator.pedestrians.seed", seed_);
    retrieveParameter(node, "pedestrian_simulator.pedestrians.single_scenario", single_scenario_, -1);
    retrieveParameter(node, "pedestrian_simulator.pedestrians.collision_free_spawn", collision_free_spawn_, true);
    retrieveParameter(node, "pedestrian_simulator.pedestrians.constant_velocity_predictions", constant_velocity_predictions_, true);
    retrieveParameter(node, "pedestrian_simulator.pedestrians.interaction", interaction_, false);

    retrieveParameter(node, "pedestrian_simulator.pedestrians.process_noise", process_noise_);
    // retrieveParameter(node, "pedestrian_simulator.pedestrians/velocity", ped_velocity_);

    retrieveParameter(node, "pedestrian_simulator.binomial.p", p_binomial_);

    retrieveParameter(node, "pedestrian_simulator.static", static_, false);

    origin_R_ = Eigen::Matrix2d::Identity();

    // Define the pedestrian type (string -> enum class)
    retrieveParameter(node, "pedestrian_simulator.pedestrians.type", ped_type_string_);
    if (ped_type_string_.compare("waypoint") == 0)
      ped_type_ = PedestrianType::WAYPOINT;
    else if (ped_type_string_.compare("gaussian") == 0)
      ped_type_ = PedestrianType::GAUSSIAN;
    else if (ped_type_string_.compare("binomial") == 0)
      ped_type_ = PedestrianType::BINOMIAL;

    // retrieveParameter(node, "pedestrian_simulator.external_waypoint_topic", external_waypoint_topic_, std::string());

    success_ = true;

    if (debug_output_)
      RCLCPP_INFO(node->get_logger(), "[Pedestrian Simulator]: Configuration initialized");
  }
};