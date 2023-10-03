#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace pedestrian_simulator
{
#define CONFIG Config::Get()

  enum class PedestrianType
  {
    WAYPOINT = 0,
    GAUSSIAN = 1,
    BINOMIAL = 2,
    SOCIAL = 3
  };

  class Config
  {

    /**
     * @brief Class for retrieving configuration parameters
     *
     */

  public:
    // Singleton function
    static Config &Get()
    {

      static Config instance_;

      return instance_;
    }

    Config() { success_ = false; };
    Config(const Config &) = delete;

    ~Config();

    void Init(rclcpp::Node *node);

    // High-level Parameters
    bool debug_output_;
    double update_frequency_;
    double delta_t_;
    bool pretend_to_be_optitrack_;

    Eigen::Matrix2d origin_R_;

    double prediction_step_;
    double horizon_N_;

    std::string scenario_file_;

    bool use_path_origin_;

    int seed_;
    int single_scenario_;
    std::vector<double> process_noise_;
    double ped_velocity_;
    std::string ped_type_string_;
    PedestrianType ped_type_;
    bool constant_velocity_predictions_;
    bool interaction_;
    double ped_radius_;

    bool collision_free_spawn_;

    double p_binomial_;
    bool static_;

    bool success_;

    double social_strength_;

  public:
    /* Retrieve paramater, if it doesn't exist return false */
    template <class T>
    static bool retrieveParameter(rclcpp::Node *node, const std::string &name, T &value)
    {
      node->declare_parameter<T>(name);
      node->get_parameter(name, value);

      if (!node->get_parameter(name, value))
      {
        RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << " not set on node " << node->get_name());
        return false;
      }
      else
      {
        return true;
      }
    }

    /* Retrieve parameter, if it doesn't exist use the default */
    template <class T>
    static void retrieveParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
    {
      node->declare_parameter<T>(name, default_value);
      node->get_parameter(name, value);

      if (!node->get_parameter(name, value))
      {
        RCLCPP_WARN_STREAM(node->get_logger(), " Setting " << name << " to default value: " << default_value);
        value = default_value;
      }
    }
  };
};
#endif
