#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>

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

    void Transform(const geometry_msgs::Pose &frame)
    {
        std::cout << "x: " << x << ", y: " << y << std::endl;
        Eigen::Matrix2d R = Helpers::rotationMatrixFromHeading(Helpers::quaternionToAngle(frame.orientation));

        Eigen::Vector2d pos(x, y);
        Eigen::Vector2d result = R * pos;

        x = result(0) + frame.position.x;
        y = result(1) + frame.position.y;
        std::cout << "x: " << x << ", y: " << y << std::endl;
    }

    void UndoTransform(const geometry_msgs::Pose &frame)
    {
        std::cout << "x: " << x << ", y: " << y << std::endl;

        Eigen::Matrix2d R = Helpers::rotationMatrixFromHeading(-Helpers::quaternionToAngle(frame.orientation));

        Eigen::Vector2d pos(x, y);
        Eigen::Vector2d result = R * pos;

        x = result(0) - frame.position.x;
        y = result(1) - frame.position.y;
        std::cout << "x: " << x << ", y: " << y << std::endl;
    }
};

typedef std::vector<Waypoint> Path;

#endif // __TYPES_H__