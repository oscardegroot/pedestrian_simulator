
#include <rclcpp/rclcpp.hpp>

#include "pedestrian_simulator.h"

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        // auto node = rclcpp::Node::make_shared("talker");
        PedestrianSimulator simulator;

        // spin node, till ROS node is running on
        // ROS_INFO_STREAM_NAMED("%s INITIALIZE SUCCESSFULLY!!", ros::this_node::getName().c_str());
        rclcpp::spin(simulator);
    }

    catch (ros::Exception &e)
    {
        // ROS_ERROR("predictive_control_node: Error occured: %s ", e.what());
        exit(1);
    }

    return 0;
}