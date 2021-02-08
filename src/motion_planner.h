#pragma once

#include "path_planner.h"
#include "sensor_fusion.h"
#include "world.h"

class MotionPlanner
{
public:
    MotionPlanner(const World& world) : world_{world}, path_planner_{world} {}
    void Update(const std::vector<std::vector<double>>& sensor_fusion_raw,
                const ControlTrajectory& old_trajectory, const CarState& car_state);
    ControlTrajectory GetOptimalTrajectory() const;

private:
    const World& world_;
    ControlTrajectory old_trajectory_{};
    PathPlanner path_planner_;
    SensorFusion sensor_fusion_{};
    double target_velocity_{0.};
    CarState ego_state_{};
    void UpdateVelocity();
};
