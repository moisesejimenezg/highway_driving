#include "path_planner.h"

#include "helpers.h"

namespace
{
ControlTrajectory GetStraightLine(double x, double y, double yaw)
{
    constexpr auto jump{0.45};
    ControlTrajectory result;
    for (auto i{0u}; i < 50; ++i)
    {
        result.x.emplace_back(x + i * jump * cos(deg2rad(yaw)));
        result.y.emplace_back(y + i * jump * sin(deg2rad(yaw)));
    }
    return result;
}
}  // namespace

ControlTrajectory PathPlanner::GetControlTrajectory(double x, double y, double yaw) const
{
    return GetStraightLine(x, y, yaw);
}
