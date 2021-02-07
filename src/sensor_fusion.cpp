#include "sensor_fusion.h"

void SensorFusion::Update(const std::vector<std::vector<double>>& raw_data)
{
    for (const auto car_raw_data : raw_data)
    {
        objects_.emplace_back(
            Object{car_raw_data[3], car_raw_data[4], car_raw_data[5], car_raw_data[6]});
    }
}
