#include "path_planner.h"

#include "helpers.h"
#include "spline.h"

namespace
{
ControlTrajectory GetStraightLine(const CarState& car_state)
{
    constexpr auto jump{0.45};
    ControlTrajectory result;
    for (auto i{0u}; i < 50; ++i)
    {
        result.x.emplace_back(car_state.x + i * jump * cos(deg2rad(car_state.yaw)));
        result.y.emplace_back(car_state.y + i * jump * sin(deg2rad(car_state.yaw)));
    }
    return result;
}

PathPlanner::History GetHistory(const ControlTrajectory& old_trajectory, const CarState& car_state)
{
    ControlTrajectory previous_trajectory{};
    const auto& size{old_trajectory.x.size()};
    auto previous_yaw{deg2rad(car_state.yaw)};
    auto previous_x = car_state.x - cos(car_state.yaw);
    auto previous_y = car_state.y - sin(car_state.yaw);
    auto x = car_state.x;
    auto y = car_state.y;
    if (size >= 2u)
    {
        previous_x = old_trajectory.x[size - 2];
        previous_y = old_trajectory.y[size - 2];
        x = old_trajectory.x[size - 1];
        y = old_trajectory.y[size - 1];
        previous_yaw = atan2(y - previous_y, x - previous_x);
    }
    previous_trajectory.x.push_back(previous_x);
    previous_trajectory.y.push_back(previous_y);
    previous_trajectory.x.push_back(x);
    previous_trajectory.y.push_back(y);
    return {previous_trajectory, previous_yaw};
}

std::pair<double, double> TransformFromOdometry(const double& x, const double& y,
                                                const double& reference_x,
                                                const double& reference_y,
                                                const double& reference_yaw)
{
    const auto shitf_x{x - reference_x};
    const auto shitf_y{y - reference_y};
    const auto& transformed_x{shitf_x * cos(0 - reference_yaw) - shitf_y * sin(0 - reference_yaw)};
    const auto& transformed_y{shitf_x * sin(0 - reference_yaw) + shitf_y * cos(0 - reference_yaw)};
    return {transformed_x, transformed_y};
}

ControlTrajectory TransformReferencePath(const ControlTrajectory& trajectory,
                                         PathPlanner::History history)
{
    const auto& reference_x{history.first.x[1]};
    const auto& reference_y{history.first.y[1]};
    const auto& reference_yaw{history.second};
    ControlTrajectory result{};
    for (auto i{0u}; i < trajectory.x.size(); ++i)
    {
        const auto& transformed_point{TransformFromOdometry(
            trajectory.x[i], trajectory.y[i], reference_x, reference_y, reference_yaw)};
        result.x.push_back(transformed_point.first);
        result.y.push_back(transformed_point.second);
    }
    return result;
}

std::pair<double, double> TransformToOdometry(const double& x, const double& y, const double& yaw)
{
    return {x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw)};
}

auto CalculateStepSize(const double& target_x, const tk::spline& spline,
                       const double& target_velocity)
{
    const auto target_y{spline(target_x)};
    const auto target_d{sqrt(target_x * target_x + target_y * target_y)};
    const auto N{target_d / (.02 * target_velocity / 2.24)};
    return target_x / N;
}

ControlTrajectory GenerateTrajectory(const ControlTrajectory& previous_trajectory,
                                     const ControlTrajectory& reference_path,
                                     const PathPlanner::History& history,
                                     const double& target_velocity)
{
    const auto& previous_x{history.first.x[1]};
    const auto& previous_y{history.first.y[1]};
    const auto& previous_yaw{history.second};
    ControlTrajectory result{};
    result.x.insert(result.x.begin(), previous_trajectory.x.begin(), previous_trajectory.x.end());
    result.y.insert(result.y.begin(), previous_trajectory.y.begin(), previous_trajectory.y.end());

    tk::spline spline{};
    spline.set_points(reference_path.x, reference_path.y);
    const auto increment{CalculateStepSize(50., spline, target_velocity)};

    auto x{0.};
    for (auto i{0u}; i <= 50 - previous_trajectory.x.size(); ++i)
    {
        x += increment;
        auto y_point = spline(x);

        auto next_point{TransformToOdometry(x, y_point, previous_yaw)};

        result.x.push_back(next_point.first + previous_x);
        result.y.push_back(next_point.second + previous_y);
    }
    return result;
}
}  // namespace

ControlTrajectory PathPlanner::GetControlTrajectory(const ControlTrajectory& previous_trajectory,
                                                    const CarState& car_state, const Lane& lane,
                                                    const double& target_velocity) const
{
    const auto straight_trajectory{GetStraightLine(car_state)};
    const auto follow_lane{
        GetLaneFollowingTrajectory(car_state, previous_trajectory, lane, target_velocity)};
    return follow_lane;
}

ControlTrajectory PathPlanner::BuildReferencePath(const CarState& car_state, const History& history,
                                                  const Lane& lane) const
{
    ControlTrajectory reference_path{};
    reference_path.x.insert(reference_path.x.begin(), history.first.x.begin(),
                            history.first.x.end());
    reference_path.y.insert(reference_path.y.begin(), history.first.y.begin(),
                            history.first.y.end());
    constexpr auto jump{30.};
    const auto d{lane.GetCenterD()};
    for (auto i{0u}; i < 3; ++i)
    {
        const auto s{car_state.s + (i + 1) * jump};
        const auto xy{
            getXY(s, d, world_.map_waypoints_s, world_.map_waypoints_x, world_.map_waypoints_y)};
        reference_path.x.emplace_back(xy[0]);
        reference_path.y.emplace_back(xy[1]);
    }
    return reference_path;
}

ControlTrajectory PathPlanner::GetLaneFollowingTrajectory(
    const CarState& car_state, const ControlTrajectory& previous_trajectory, const Lane& lane,
    const double& target_velocity) const
{
    const auto& history{GetHistory(previous_trajectory, car_state)};
    const auto& reference_path{BuildReferencePath(car_state, history, lane)};
    const auto& transformed_reference_path{TransformReferencePath(reference_path, history)};
    return GenerateTrajectory(previous_trajectory, transformed_reference_path, history,
                              target_velocity);
}
