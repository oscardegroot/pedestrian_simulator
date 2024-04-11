#include <pedestrian_simulator/spawn_randomizer.h>

#include <ros_tools/random_generator.h>

double Range::GenerateRandom(RosTools::RandomGenerator *random_generator)
{
    return min + random_generator->Double() * (max - min);
}

double Range::GenerateNormalRandom(RosTools::RandomGenerator *random_generator)
{
    // Assumed that max is at +1sigma and min is at -1sigma
    double mean = (max + min) / 2.;
    return random_generator->Gaussian(mean, max - mean);
    // min + random_generator->Double() * (max - min);
}

Range Range::Inflate(double inflation_mp)
{
    double mean = (max + min) / 2.;
    return Range(mean + (min - mean) * inflation_mp, mean + (max - mean) * inflation_mp);
}

SpawnRandomizer::SpawnRandomizer()
{
    goal_offset_ = Waypoint(0., 0.);
};

Waypoint SpawnRandomizer::GenerateStart(RosTools::RandomGenerator *random_generator)
{
    return Waypoint(range_x_.GenerateRandom(random_generator), range_y_.GenerateRandom(random_generator));
}

Waypoint SpawnRandomizer::GenerateGoal(RosTools::RandomGenerator *random_generator)
{
    Waypoint absolute = Waypoint(range_x_.Inflate(goal_inflation_).GenerateRandom(random_generator),
                                 range_y_.Inflate(goal_inflation_).GenerateRandom(random_generator));

    return Waypoint(absolute.x + goal_offset_.x, absolute.y + goal_offset_.y);
}

double SpawnRandomizer::GenerateVelocity(RosTools::RandomGenerator *random_generator)
{
    return range_v_.GenerateNormalRandom(random_generator);
}

void SpawnRandomizer::ReadFrom(rapidxml::xml_node<> *tag)
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

    auto *goal_range_tag = tag->first_node("goal_range");
    if (goal_range_tag)
        goal_range_ = atof(goal_range_tag->first_attribute("value")->value());
    else
        goal_range_ = 3.;

    auto *travel_time_tag = tag->first_node("travel_time");
    if (travel_time_tag)
        min_travel_time_ = atof(travel_time_tag->first_attribute("value")->value());
    else
        min_travel_time_ = 5.;

    // The goal inflation tag enlarges the region that the goal can be in
    auto *goal_inflation = tag->first_node("goal_inflation");
    if (goal_inflation)
        goal_inflation_ = atof(goal_inflation->first_attribute("value")->value());
}

void SpawnRandomizer::ReadRange(rapidxml::xml_node<> *tag, const std::string &&name, Range &range)
{
    auto *range_tag = tag->first_node(name.c_str());
    range = Range(atof(range_tag->first_attribute("min")->value()),
                  atof(range_tag->first_attribute("max")->value()));
}