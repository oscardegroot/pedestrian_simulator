#ifndef __WAYPOINT_PEDESTRIAN_H__
#define __WAYPOINT_PEDESTRIAN_H__

#include <pedestrians/pedestrian.h>
namespace pedestrian_simulator
{
    /** @deprecated Follow waypoints deterministically */
    class WaypointPedestrian : public Pedestrian
    {
    public:
        WaypointPedestrian(const Waypoint &start, double velocity);

    public:
        unsigned int current_path_id_, current_waypoint_id_;
        std::vector<Path> paths_;

        void Reset() override;

        void Update() override;

        // Set the waypoint to the closest one
        bool FindClosestWaypoint();

        void PickPath();

        bool HasReachedWaypoint();

        Waypoint &GetCurrentWaypoint();

    protected:
        bool done_;

        RosTools::RandomGenerator random_generator;
    };
};
#endif // __WAYPOINT_PEDESTRIAN_H__