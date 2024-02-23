
#include <ros/ros.h>

#include "pedestrian_simulator.h"

#include <memory>

std::unique_ptr<PedestrianSimulator> simulator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    simulator.reset(new PedestrianSimulator());

    ros::spin();

    return 0;
}
