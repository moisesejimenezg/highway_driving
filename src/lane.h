#pragma once

enum class LaneId
{
    // clang-format off
    kInvalid = -1,
    kLeft    =  0,
    kCenter  =  1,
    kRight   =  2
    // clang-format on
};

class Lane
{
public:
    Lane(const double& d);
    LaneId GetLaneId() const { return lane_id_; }
    double GetCenterD() const;

private:
    LaneId lane_id_{};
};

