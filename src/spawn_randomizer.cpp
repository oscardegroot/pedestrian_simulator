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

Waypoint SpawnRandomizer::GeneratePointInRectangle(RosTools::RandomGenerator *random_generator)
{
    // Generate two random barycentric coordinates
    double r1 = random_generator->Double();
    double r2 = random_generator->Double();

    // Ensure that the sum of r1 and r2 doesn't exceed 1 by swapping if necessary
    if (r1 + r2 > 1)
    {
        r1 = 1 - r1;
        r2 = 1 - r2;
    }

    // Compute the random point using barycentric coordinates
    Waypoint result;
    result.x = (1 - r1 - r2) * square_points_[0](0) + r1 * square_points_[1](0) + r2 * square_points_[2](0);
    result.y = (1 - r1 - r2) * square_points_[0](1) + r1 * square_points_[1](1) + r2 * square_points_[2](1);
    return result;
}

Waypoint SpawnRandomizer::GenerateStart(RosTools::RandomGenerator *random_generator)
{
    if (!use_square_)
    {
        return Waypoint(range_x_.GenerateRandom(random_generator), range_y_.GenerateRandom(random_generator));
    }
    else
    {
        return GeneratePointInRectangle(random_generator);
    }
}

Waypoint SpawnRandomizer::GenerateGoal(RosTools::RandomGenerator *random_generator)
{
    if (!use_square_)
    {
        Waypoint absolute = Waypoint(range_x_.Inflate(goal_inflation_).GenerateRandom(random_generator),
                                     range_y_.Inflate(goal_inflation_).GenerateRandom(random_generator));

        return Waypoint(absolute.x + goal_offset_.x, absolute.y + goal_offset_.y);
    }
    else
    {
        return GeneratePointInRectangle(random_generator);
    }
}

double SpawnRandomizer::GenerateVelocity(RosTools::RandomGenerator *random_generator)
{
    return range_v_.GenerateNormalRandom(random_generator);
}

void SpawnRandomizer::ReadFromSquare(rapidxml::xml_node<> *tag)
{
    use_square_ = true; // In this case we spawn pedestrians in a random polygon defined by four points
    min_travel_time_ = 20.;
    range_v_ = Range(0, 1.); // Should not be used anyway
    for (int i = 1; i <= 4; i++)
    {

        auto *point_tag = tag->first_node(("p" + std::to_string(i)).c_str());
        square_points_.emplace_back(atof(point_tag->first_attribute("x")->value()), atof(point_tag->first_attribute("y")->value()));
    }
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