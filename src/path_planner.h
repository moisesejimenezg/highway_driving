#pragma once

#include <vector>

struct ControlTrajectory
{
    std::vector<double> x{};
    std::vector<double> y{};
};

class PathPlanner
{
public:
    ControlTrajectory GetControlTrajectory(double x, double y, double yaw) const;
};
