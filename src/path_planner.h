#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "world.h"

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
    PathPlanner(const World& world) : world_{world} {}
    ControlTrajectory GetControlTrajectory(const CarState& car_state) const;

private:
    const World& world_;
};
#endif  // PATH_PLANNER_H
