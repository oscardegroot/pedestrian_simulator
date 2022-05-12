#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include "helpers.h"

#define CONFIG Config::Get()

enum class PedestrianType
{
  WAYPOINT = 0,
  GAUSSIAN = 1,
  BINOMIAL = 2
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
  std::string scenario_file_;

  int seed_;
  std::vector<double> process_noise_;
  double ped_velocity_;
  std::string ped_type_string_;
  PedestrianType ped_type_;

  double p_binomial_;
  bool static_;

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
