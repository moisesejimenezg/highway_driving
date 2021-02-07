#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "car_state.h"
#include "lane.h"
#include "world.h"

struct ControlTrajectory
{
    std::vector<double> x{};
    std::vector<double> y{};
};

class PathPlanner
{
public:
    using History = std::pair<ControlTrajectory, double>;
    PathPlanner(const World& world) : world_{world} {}
    ControlTrajectory GetControlTrajectory(const ControlTrajectory& old_trajectory,
                                           const CarState& car_state, const Lane& lane,
                                           const double& target_velocity) const;

private:
    const World& world_;
    ControlTrajectory BuildReferencePath(const CarState&, const History&, const Lane&) const;
    ControlTrajectory GetLaneFollowingTrajectory(const CarState& car_state,
                                                 const ControlTrajectory& previous_trajectory,
                                                 const Lane& lane,
                                                 const double& target_velocity) const;
};
#endif  // PATH_PLANNER_H
