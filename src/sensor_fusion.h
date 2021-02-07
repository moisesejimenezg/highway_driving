#pragma once
#include <vector>

struct Object
{
    double vx{};
    double vy{};
    double s{};
    double d{};
};

class SensorFusion
{
public:
    void Update(const std::vector<std::vector<double>>&);

private:
    std::vector<Object> objects_{};
};
