#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

struct CarState
{
    double x{};
    double y{};
    double s{};
    double d{};
    double yaw{};
    double v{};
};

struct ControlTrajectory
{
    std::vector<double> x{};
    std::vector<double> y{};
};

class PathPlanner
{
public:
    ControlTrajectory GetControlTrajectory(const CarState& car_state) const;
};
#endif  // PATH_PLANNER_H
