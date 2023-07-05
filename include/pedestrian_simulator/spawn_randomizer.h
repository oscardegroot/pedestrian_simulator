#ifndef __SPAWN_RANDOMIZER_H__
#define __SPAWN_RANDOMIZER_H__

#include <pedestrian_simulator/types.h>

#include <rapidxml_utils.hpp>
#include <ros_tools/helpers.h>

#include <string>

struct Range
{
    double min;
    double max;

    Range(){};
    Range(double _min, double _max)
    {
        min = _min;
        max = _max;
    }

    double GenerateRandom(RosTools::RandomGenerator *random_generator);

    double GenerateNormalRandom(RosTools::RandomGenerator *random_generator);

    Range Inflate(double inflation_mp);
};

class SpawnRandomizer
{
public:
    SpawnRandomizer();

    Waypoint GenerateStart(RosTools::RandomGenerator *random_generator);
    Waypoint GenerateGoal(RosTools::RandomGenerator *random_generator);
    double GenerateVelocity(RosTools::RandomGenerator *random_generator);

public:
    void ReadFrom(rapidxml::xml_node<> *tag);

private:
    Range range_x_, range_y_, range_v_;
    Waypoint goal_offset_;
    double goal_inflation_ = 1.;

    void ReadRange(rapidxml::xml_node<> *tag, const std::string &&name, Range &range);
};

#endif // __SPAWN_RANDOMIZER_H__