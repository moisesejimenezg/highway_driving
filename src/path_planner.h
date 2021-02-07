#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "world.h"

struct CarState
{
    double x{};
    double y{};
    double s{};
    double d{};
    double yaw{};
    double v{};
};

struct ControlTrajectory
{
    std::vector<double> x{};
    std::vector<double> y{};
};

enum class LaneId
{
    // clang-format off
    kInvalid = -1,
    kLeft    =  0,
    kCenter  =  1,
    kRight   =  2
    // clang-format on
};

LaneId GetLaneId(double d);
double GetCenterD(LaneId);

class PathPlanner
{
public:
    using History = std::pair<ControlTrajectory, double>;
    PathPlanner(const World& world) : world_{world} {}
    ControlTrajectory GetControlTrajectory(const ControlTrajectory& old_trajectory,
                                           const CarState& car_state, const LaneId& lane_id) const;

private:
    const World& world_;
    ControlTrajectory BuildReferencePath(const CarState&, const History&, const LaneId&) const;
    ControlTrajectory GetLaneFollowingTrajectory(const CarState& car_state,
                                                 const ControlTrajectory& previous_trajectory,
                                                 const LaneId& lane_id) const;
};
#endif  // PATH_PLANNER_H
