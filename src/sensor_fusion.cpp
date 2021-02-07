#include "sensor_fusion.h"

#include "algorithm"
#include "lane.h"

void SensorFusion::Update(const std::vector<std::vector<double>>& raw_data)
{
    objects_.clear();
    for (const auto car_raw_data : raw_data)
    {
        objects_.emplace_back(
            Object{car_raw_data[3], car_raw_data[4], car_raw_data[5], car_raw_data[6]});
    }
    std::sort(objects_.begin(), objects_.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.s < rhs.s; });
}

std::optional<Object> SensorFusion::GetObjectInFront(const CarState& ego) const
{
    const Lane ego_lane{ego.d};
    std::optional<Object> result{};
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        // TODO: Might have to project car one step ahead
        if (object_lane.GetLaneId() == ego_lane.GetLaneId() && object.s > ego.s &&
            object.s - ego.s <= 30)
        {
            result = object;
            break;
        }
    }
    return result;
}
