#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Eigen/Dense>
#include <ros/ros.h>

#include <string>
#include <vector>

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

  void Init();

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

  double social_strength_, social_decay_, social_l_;

  bool success_;

public:
  /* Retrieve paramater, if it doesn't exist return false */
  template <class T>
  static bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
  {

    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
      return false;
    }
    else
    {
      return true;
    }
  }

  /* Retrieve parameter, if it doesn't exist use the default */
  template <class T>
  static void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
  {

    if (!retrieveParameter(nh, name, value))
    {
      ROS_WARN_STREAM(" Setting " << name << " to default value: " << default_value);
      value = default_value;
    }
  }
};

#endif
