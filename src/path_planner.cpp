#include "path_planner.h"

#include "helpers.h"

namespace
{
ControlTrajectory GetStraightLine(const CarState& car_state)
{
    constexpr auto jump{0.45};
    ControlTrajectory result;
    for (auto i{0u}; i < 50; ++i)
    {
        result.x.emplace_back(car_state.x + i * jump * cos(deg2rad(car_state.yaw)));
        result.y.emplace_back(car_state.y + i * jump * sin(deg2rad(car_state.yaw)));
    }
    return result;
}

ControlTrajectory GetLaneFollowingLine(const CarState& car_state, const World& world)
{
    constexpr auto jump{0.40};
    ControlTrajectory result;
    for (auto i{0u}; i < 50; ++i)
    {
        const auto s{car_state.s + (i + 1) * jump};
        const auto d{6.};
        const auto xy{
            getXY(s, d, world.map_waypoints_s, world.map_waypoints_x, world.map_waypoints_y)};
        result.x.emplace_back(xy[0]);
        result.y.emplace_back(xy[1]);
    }
    return result;
}
}  // namespace

ControlTrajectory PathPlanner::GetControlTrajectory(const CarState& car_state) const
{
    return GetLaneFollowingLine(car_state, world_);
}
