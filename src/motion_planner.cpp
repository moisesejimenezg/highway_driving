#include "motion_planner.h"

constexpr double kMaximumVelocity = 44.5;

void MotionPlanner::Update(const std::vector<std::vector<double>> &sensor_fusion_raw,
                           const ControlTrajectory &old_trajectory, const CarState &car_state)
{
    ego_state_ = car_state;
    target_lane_id_ = LaneId::kCenter;
    old_trajectory_ = old_trajectory;
    sensor_fusion_.Update(sensor_fusion_raw);
    UpdateVelocity();
}

ControlTrajectory MotionPlanner::GetOptimalTrajectory() const
{
    const Lane lane{LaneId::kCenter};
    return path_planner_.GetControlTrajectory(old_trajectory_, ego_state_, lane, target_velocity_);
}

void MotionPlanner::UpdateVelocity()
{
    const auto acceleration{0.224};
    const auto object_in_front{sensor_fusion_.GetObjectInFront(ego_state_)};
    if (object_in_front.has_value())
    {
        target_velocity_ -= acceleration;
    }
    else if (target_velocity_ < kMaximumVelocity)
    {
        target_velocity_ += acceleration;
    }
}
