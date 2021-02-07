#pragma once
#include <cmath>
#include <optional>
#include <vector>

#include "car_state.h"

struct Object
{
    double vx{};
    double vy{};
    double s{};
    double d{};
    double GetV() const { return sqrt(vx * vx + vy * vy); }
};

class SensorFusion
{
public:
    void Update(const std::vector<std::vector<double>>&);
    std::optional<Object> GetObjectInFront(const CarState& ego) const;

private:
    std::vector<Object> objects_{};
};
