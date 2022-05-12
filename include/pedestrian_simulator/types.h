#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>

// constexpr double DELTA_T = 0.05;
constexpr double DELTA_T_PREDICT = 0.2;
constexpr int HORIZON_N = 20;

struct Waypoint
{
    double x, y;

    Waypoint(){};

    Waypoint(double x, double y)
        : x(x), y(y){};

    double Distance(Waypoint &other)
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    double Angle(Waypoint &other)
    {
        return std::atan2(other.y - y, other.x - x);
    }

    Waypoint Translate(const geometry_msgs::Pose &frame)
    {
        x -= frame.position.x;
        y -= frame.position.y;
        return *this;
    }
};

typedef std::vector<Waypoint> Path;

#endif // __TYPES_H__