#include <pedestrians/waypoint_pedestrian.h>

#include <pedestrian_simulator/configuration.h>

WaypointPedestrian::WaypointPedestrian(const Waypoint &start, double velocity)
    : Pedestrian(start, velocity)
{
}

void WaypointPedestrian::Reset()
{
    Pedestrian::Reset();

    current_waypoint_id_ = 0;
    done_ = false;

    PickPath();
}

void WaypointPedestrian::Update()
{
    if (done_)
        return;

    // WAYPOINT BASED (I cannot compute the probability distribution for this)
    // Check if ped has reach the waypoint
    if (HasReachedWaypoint())
    {
        if (current_waypoint_id_ < paths_[current_path_id_].size() - 1)
            current_waypoint_id_++;
        else
        {
            // Stop moving
            done_ = true;
            twist_.linear.x = 0.;
            twist_.linear.y = 0.;
        }
    }

    // Move towards the waypoint
    Waypoint &cur_waypoint = GetCurrentWaypoint();
    twist_.linear.x = velocity_ * std::cos(position_.Angle(cur_waypoint));
    twist_.linear.y = velocity_ * std::sin(position_.Angle(cur_waypoint));

    position_.x += twist_.linear.x * CONFIG.delta_t_;
    position_.y += twist_.linear.y * CONFIG.delta_t_;
    // std::cout << "x = " << position_.x << ", y = " << position_.y << std::endl;
}

// Set the waypoint to the closest one
bool WaypointPedestrian::FindClosestWaypoint()
{
    double min_dist = 1e9;
    double direction = std::atan2(twist_.linear.y, twist_.linear.x);
    for (size_t waypoint_id = 0; waypoint_id < paths_[current_path_id_].size(); waypoint_id++)
    {
        double cur_dist = position_.Distance(paths_[current_path_id_][waypoint_id]);
        double angle = position_.Angle(paths_[current_path_id_][waypoint_id]);
        double angle_diff = std::abs(angle - direction);

        while (angle_diff > (2 * M_PI))
            angle_diff -= 2 * M_PI;

        // If we are already at the waypoint (i.e., direction doesn't matter) OR we are moving towards the waypoint
        if ((done_ || angle_diff < M_PI_2) && cur_dist < min_dist)
        {
            min_dist = cur_dist;
            current_waypoint_id_ = waypoint_id;
        }
    }
    if (min_dist == 1e9)
        return false;
    else
        return true;
}

void WaypointPedestrian::PickPath()
{
    current_path_id_ = random_generator.Int(paths_.size());
}

bool WaypointPedestrian::HasReachedWaypoint()
{
    return position_.Distance(GetCurrentWaypoint()) < 0.05;
}

Waypoint &WaypointPedestrian::GetCurrentWaypoint()
{
    return paths_[current_path_id_][current_waypoint_id_];
}