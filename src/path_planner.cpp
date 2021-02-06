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
}  // namespace

ControlTrajectory PathPlanner::GetControlTrajectory(const CarState& car_state) const
{
    return GetStraightLine(car_state);
}
