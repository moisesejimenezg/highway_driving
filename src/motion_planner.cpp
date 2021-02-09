#include "motion_planner.h"

void MotionPlanner::Update(const std::vector<std::vector<double>> &sensor_fusion_raw,
                           const ControlTrajectory &old_trajectory, const CarState &car_state)
{
    ego_state_ = car_state;
    old_trajectory_ = old_trajectory;
    sensor_fusion_.Update(sensor_fusion_raw, car_state);
    UpdateVelocity();
    UpdateTargetLane();
}

ControlTrajectory MotionPlanner::GetOptimalTrajectory() const
{
    return path_planner_.GetControlTrajectory(old_trajectory_, ego_state_, Lane{target_lane_id_},
                                              target_velocity_);
}

constexpr auto kAcceleration{0.224};
constexpr auto kMetersPerSecondToMilesPerHour{2.237};
constexpr auto kMaximumVelocityInMilesPerHour{49.5};
void MotionPlanner::UpdateVelocity()
{
    const auto object_in_front{sensor_fusion_.GetObjectInFront(ego_state_)};
    if (object_in_front.has_value() &&
        object_in_front.value().GetV() * kMetersPerSecondToMilesPerHour < target_velocity_)
    {
        target_velocity_ -= kAcceleration;
    }
    else if (target_velocity_ < kMaximumVelocityInMilesPerHour)
    {
        target_velocity_ += kAcceleration;
    }
}

void MotionPlanner::UpdateTargetLane()
{
    const auto object_in_front{sensor_fusion_.GetObjectInFront(ego_state_)};
    const auto ego_lane{Lane(ego_state_.d)};
    target_lane_id_ = ego_lane.GetLaneId();
    if (object_in_front.has_value())
    {
        const auto object_to_the_left{sensor_fusion_.GetNeighborToTheLeft()};
        const auto object_to_the_right{sensor_fusion_.GetNeighborToTheRight()};
        if (ego_lane.GetLaneId() != LaneId::kLeft && !object_to_the_left)
        {
            target_lane_id_ = static_cast<LaneId>(static_cast<int>(ego_lane.GetLaneId()) - 1);
        }
        if (ego_lane.GetLaneId() != LaneId::kRight && !object_to_the_right)
        {
            target_lane_id_ = static_cast<LaneId>(static_cast<int>(ego_lane.GetLaneId()) + 1);
        }
    }
}
