#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>

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

    Waypoint Translate(const geometry_msgs::Pose &frame)
    {
        x -= frame.position.x;
        y -= frame.position.y;
        return *this;
    }

    /** @brief Transform a position to the position of the given frame, rotated with the given angle (in radians) */
    void Transform(const geometry_msgs::Pose &frame, double angle)
    {
        Eigen::Matrix2d R = RosTools::rotationMatrixFromHeading(-angle); // Rotation matrix

        Eigen::Vector2d transform_pos(frame.position.x, frame.position.y); // Frame (x, y)
        transform_pos = transform_pos + R * Eigen::Vector2d(x, y);         // Frame + rotated position

        x = transform_pos(0); // Save
        y = transform_pos(1);
    }

    void UndoTransform(const geometry_msgs::Pose &frame)
    {
        x = og_x; // Reset the x, y
        y = og_y;
    }
};
/** @note Simple struct to simplify passing the robot state */
struct RobotState
{
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;

    RobotState(){};

    RobotState(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pos(0) = msg->pose.position.x;
        pos(1) = msg->pose.position.y;

        vel(0) = std::cos(msg->pose.orientation.z) * msg->pose.position.z;
        vel(1) = std::sin(msg->pose.orientation.z) * msg->pose.position.z;
    }
};

typedef std::vector<Waypoint> Path;

#endif // __TYPES_H__