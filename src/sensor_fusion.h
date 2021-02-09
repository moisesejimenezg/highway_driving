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
    void Update(const std::vector<std::vector<double>>&, const CarState&);
    std::optional<Object> GetObjectInFront(const CarState& ego) const { return object_in_front_; }
    std::optional<Object> GetNeighborToTheLeft() const { return object_to_the_left_; }
    std::optional<Object> GetNeighborToTheRight() const { return object_to_the_right_; }

private:
    static constexpr double safety_buffer_{30.};
    std::vector<Object> objects_{};
    std::optional<Object> object_in_front_{};
    std::optional<Object> object_to_the_left_{};
    std::optional<Object> object_to_the_right_{};
    CarState ego_state_{};

    void Reset();
    void UpdateObjects(const std::vector<std::vector<double>>&);
    void UpdateObjectInFront();
    void UpdateObjectToTheLeft();
    void UpdateObjectToTheRight();
};
