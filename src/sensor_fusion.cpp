#include "sensor_fusion.h"

#include "algorithm"
#include "lane.h"

void SensorFusion::Update(const std::vector<std::vector<double>>& raw_data,
                          const CarState& ego_state)
{
    Reset();
    ego_state_ = ego_state;
    UpdateObjects(raw_data);
    UpdateObjectInFront();
    UpdateObjectToTheLeft();
    UpdateObjectToTheRight();
}

void SensorFusion::Reset()
{
    objects_.clear();
    object_in_front_.reset();
    object_to_the_left_.reset();
    object_to_the_right_.reset();
}

void SensorFusion::UpdateObjects(const std::vector<std::vector<double>>& raw_data)
{
    for (const auto car_raw_data : raw_data)
    {
        objects_.emplace_back(
            Object{car_raw_data[3], car_raw_data[4], car_raw_data[5], car_raw_data[6]});
    }
    std::sort(objects_.begin(), objects_.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.s < rhs.s; });
}

void SensorFusion::UpdateObjectInFront()
{
    const Lane ego_lane{ego_state_.d};
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        // TODO: Might have to project car one step ahead
        if (object_lane.GetLaneId() == ego_lane.GetLaneId() && object.s > ego_state_.s &&
            object.s - ego_state_.s <= safety_buffer_)
        {
            object_in_front_ = object;
            break;
        }
    }
}

void SensorFusion::UpdateObjectToTheLeft()
{
    const Lane ego_lane{ego_state_.d};
    if (ego_lane.GetLaneId() == LaneId::kLeft)
    {
        return;
    }
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        if (static_cast<int>(object_lane.GetLaneId()) < static_cast<int>(ego_lane.GetLaneId()) &&
            object.s > ego_state_.s && object.s - ego_state_.s <= safety_buffer_)
        {
            object_to_the_left_ = object;
            break;
        }
    }
}

void SensorFusion::UpdateObjectToTheRight()
{
    const Lane ego_lane{ego_state_.d};
    if (ego_lane.GetLaneId() == LaneId::kRight)
    {
        return;
    }
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        if (static_cast<int>(object_lane.GetLaneId()) > static_cast<int>(ego_lane.GetLaneId()) &&
            object.s > ego_state_.s && object.s - ego_state_.s <= safety_buffer_)
        {
            object_to_the_right_ = object;
            break;
        }
    }
}
