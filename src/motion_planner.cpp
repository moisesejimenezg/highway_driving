#include "motion_planner.h"

void MotionPlanner::Update(const std::vector<std::vector<double>> &sensor_fusion_raw,
                           const ControlTrajectory &old_trajectory, const CarState &car_state)
{
    ego_state_ = car_state;
    old_trajectory_ = old_trajectory;
    sensor_fusion_.Update(sensor_fusion_raw);
    UpdateVelocity();
}

ControlTrajectory MotionPlanner::GetOptimalTrajectory() const
{
    const Lane lane{ego_state_.d};
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
    else if (target_velocity_ < 49.5)
    {
        target_velocity_ += acceleration;
    }
}
