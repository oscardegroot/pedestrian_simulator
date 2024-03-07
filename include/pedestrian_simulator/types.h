#ifndef __TYPES_H__
#define __TYPES_H__

#include <Eigen/Dense>
#include <ros_tools/math.h>

#include <vector>

struct Prediction
{
    int id{-1};
    std::vector<Eigen::Vector2d> pos;
    std::vector<double> angle;
    std::vector<Eigen::Vector2d> vel;
    std::vector<double> major_axis;
    std::vector<double> minor_axis;

    Prediction(){};

    void Add(const Eigen::Vector2d &p, const double a, const Eigen::Vector2d &v,
             const double _major_axis = 0, const double _minor_axis = 0)
    {
        pos.push_back(p);
        angle.push_back(a);
        vel.push_back(v);
        major_axis.push_back(_major_axis);
        minor_axis.push_back(_minor_axis);
    }
};

struct Waypoint
{
    double x, y;
    double og_x, og_y;

    Waypoint(){};

    Waypoint(double x, double y)
        : x(x), y(y), og_x(x), og_y(y){};

    double Distance(Waypoint &other)
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    double Angle(Waypoint &other)
    {
        return std::atan2(other.y - y, other.x - x);
    }

    // Waypoint Translate(const geometry_msgs::Pose &frame)
    // {
    //     x -= frame.position.x;
    //     y -= frame.position.y;
    //     return *this;
    // }

    // /** @brief Transform a position to the position of the given frame, rotated with the given angle (in radians) */
    // void Transform(const geometry_msgs::Pose &frame, double angle)
    // {
    //     Eigen::Matrix2d R = RosTools::rotationMatrixFromHeading(-angle); // Rotation matrix

    //     Eigen::Vector2d transform_pos(frame.position.x, frame.position.y); // Frame (x, y)
    //     transform_pos = transform_pos + R * Eigen::Vector2d(x, y);         // Frame + rotated position

    //     x = transform_pos(0); // Save
    //     y = transform_pos(1);
    // }

    // void UndoTransform(const geometry_msgs::Pose &frame)
    // {
    //     x = og_x; // Reset the x, y
    //     y = og_y;
    // }
};
/** @note Simple struct to simplify passing the robot state */
struct RobotState
{
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;

    RobotState(){};

    RobotState(const Eigen::Vector2d &pos, const double angle, double velocity)
        : pos(pos)
    {
        vel(0) = std::cos(angle) * velocity;
        vel(1) = std::sin(angle) * velocity;
    }
};

struct StaticObstacle
{
    double min_x, min_y;
    double max_x, max_y;

    StaticObstacle(double _min_x, double _min_y,
                   double _max_x, double _max_y) : min_x(_min_x), min_y(_min_y), max_x(_max_x), max_y(_max_y) {}
};

typedef std::vector<Waypoint> Path;

#endif // __TYPES_H__