#ifndef __SPAWN_RANDOMIZER_H__
#define __SPAWN_RANDOMIZER_H__

#include "rapidxml_utils.hpp"
#include "ros_tools/helpers.h"
#include "pedestrian_simulator/types.h"

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

    double GenerateRandom(RosTools::RandomGenerator *random_generator)
    {
        return min + random_generator->Double() * (max - min);
    }
};

class SpawnRandomizer
{
public:
    SpawnRandomizer()
    {
        goal_offset_ = Waypoint(0., 0.);
    };

    Waypoint GenerateStart(RosTools::RandomGenerator *random_generator)
    {
        return Waypoint(range_x_.GenerateRandom(random_generator), range_y_.GenerateRandom(random_generator));
    }

    Waypoint GenerateGoal(RosTools::RandomGenerator *random_generator)
    {
        Waypoint absolute = GenerateStart(random_generator);
        return Waypoint(absolute.x + goal_offset_.x, absolute.y + goal_offset_.y);
    }

    double GenerateVelocity(RosTools::RandomGenerator *random_generator)
    {
        return range_v_.GenerateRandom(random_generator);
    }

public:
    void ReadFrom(rapidxml::xml_node<> *tag)
    {

        ReadRange(tag, "range_x", range_x_);
        ReadRange(tag, "range_y", range_y_);
        ReadRange(tag, "range_v", range_v_);

        auto *goal_offset_tag = tag->first_node("goal_offset");
        if (goal_offset_tag)
        {
            goal_offset_ = Waypoint(atof(goal_offset_tag->first_attribute("x")->value()),
                                    atof(goal_offset_tag->first_attribute("y")->value()));
        }
    }

private:
    Range range_x_, range_y_, range_v_;
    Waypoint goal_offset_;

    void ReadRange(rapidxml::xml_node<> *tag, const std::string &&name, Range &range)
    {
        auto *range_tag = tag->first_node(name.c_str());
        range = Range(atof(range_tag->first_attribute("min")->value()),
                      atof(range_tag->first_attribute("max")->value()));
    }
};

#endif // __SPAWN_RANDOMIZER_H__