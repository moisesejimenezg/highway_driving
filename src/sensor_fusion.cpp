#include "sensor_fusion.h"

#include "lane.h"

void SensorFusion::Update(const std::vector<std::vector<double>>& raw_data)
{
    for (const auto car_raw_data : raw_data)
    {
        objects_.emplace_back(
            Object{car_raw_data[3], car_raw_data[4], car_raw_data[5], car_raw_data[6]});
    }
}

std::optional<Object> SensorFusion::GetObjectInFront(const CarState& ego) const
{
    const Lane ego_lane{ego.d};
    std::optional<Object> result{};
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        if (object_lane.GetLaneId() == ego_lane.GetLaneId() && object.s > ego.s &&
            object.s - ego.s <= 5)
        {
            if (!result.has_value())
            {
                result = object;
            }
            else if (result.value().s < object.s)
            {
                result = object;
            }
        }
    }
    return result;
}
