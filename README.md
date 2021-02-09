# Highway Driving
## Motion Planner
The overarching component in the designed architecture is the MotionPlanner. It is responsible for updating the
SensorFusion and the PathPlanner components with the incoming information and producing a drivable trajectory.

The update cycle for the motion planner can be seen in the main.cpp file

```C++
motion_planner.Update(sensor_fusion_raw, old_trajectory, car);
const auto trajectory{motion_planner.GetOptimalTrajectory()};
```

The innards of the MotionPlanner::Update function can be seen below; mainly the relaying of information to SensorFusion
and the PathPlanner.

```C++
void MotionPlanner::Update(const std::vector<std::vector<double>> &sensor_fusion_raw,
                           const ControlTrajectory &old_trajectory, const CarState &car_state)
{
    ego_state_ = car_state;
    old_trajectory_ = old_trajectory;
    sensor_fusion_.Update(sensor_fusion_raw, car_state);
    UpdateVelocity();
    UpdateTargetLane();
}
```

The maneuvers to be generated are mainly defined by a target lane and a target velocity. Lanes are represented by the
Lane struct which allows from rapid conversion between LaneId and lateral displacement (d), and viceversa.

### The World

The LaneId enum describes the three drivable lanes
```C++
enum class LaneId
{
    // clang-format off
    kInvalid = -1,
    kLeft    =  0,
    kCenter  =  1,
    kRight   =  2
    // clang-format on
};
```

The CarState struct describes the ego vehicle

```C++
struct CarState
{
    double x{};
    double y{};
    double s{};
    double d{};
    double yaw{};
    double v{};
};
```

The Object struct describes other vehicles in the road

```C++
struct Object
{
    double vx{};
    double vy{};
    double s{};
    double d{};
    double GetV() const { return sqrt(vx * vx + vy * vy); }
};
```

### Velocity Matching

To determine the target velocity we look down the road in search for other vehicles in the ego lane

```C++
void MotionPlanner::UpdateVelocity()
{
    const auto object_in_front{sensor_fusion_.GetObjectInFront(ego_state_)};
    if (object_in_front.has_value() &&
        object_in_front.value().GetV() * kMetersPerSecondToMilesPerHour < target_velocity_)
    {
        target_velocity_ -= kAcceleration;
    }
    else if (target_velocity_ < kMaximumVelocityInMilesPerHour)
    {
        target_velocity_ += kAcceleration;
    }
}
```
When the road is clear the ego vehicle accelerates up to the target maximum velocity. On the other hand, when there is
a vehicle ahead the ego vehicle matches its speed.

### Following or Changing Lanes

To select the target lane for the maneuver we once again check whether the road is obstructed or not.

```C++
void MotionPlanner::UpdateTargetLane()
{
    const auto object_in_front{sensor_fusion_.GetObjectInFront(ego_state_)};
    const auto ego_lane{Lane(ego_state_.d)};
    target_lane_id_ = ego_lane.GetLaneId();
    if (object_in_front.has_value())
    {
        const auto object_to_the_left{sensor_fusion_.GetNeighborToTheLeft()};
        const auto object_to_the_right{sensor_fusion_.GetNeighborToTheRight()};
        if (ego_lane.GetLaneId() != LaneId::kLeft && !object_to_the_left)
        {
            target_lane_id_ = static_cast<LaneId>(static_cast<int>(ego_lane.GetLaneId()) - 1);
        }
        if (ego_lane.GetLaneId() != LaneId::kRight && !object_to_the_right)
        {
            target_lane_id_ = static_cast<LaneId>(static_cast<int>(ego_lane.GetLaneId()) + 1);
        }
    }
}
```

In case of having a vehicle in front lines 45-52 pick a new target lane by converting to and from the integer
representation of lane IDs. Interesting the process has an obvious bias towards the right which could be improved upon.

## Sensor Fusion
The Sensor Fusion component preprocesses the incoming environment information (i.e. obstacles in the road) and presents
the output in a conveniently accessible manner to the MotionPlanner. During its main update cycle the SensorFusion
component clears its previous results and determines whether or not there are vehicles in the ego lane and both
neighboring lanes.

```C++
void SensorFusion::Update(const std::vector<std::vector<double>>& raw_data,
                          const CarState& ego_state)
{
    Reset();
    ego_state_ = ego_state;
    UpdateObjects(raw_data);
    UpdateObjectInFront();
    UpdateObjectToTheLeft();
    UpdateObjectToTheRight();
}
```

The objects are sorted according to their distance along the Frenet arclength coordinate (`UpdateObjects`). The
functions `UpdateObjectInFront`, `UpdateObjectToTheLeft` and `UpdateObjectToTheRight` search for the nearest object that
is within a set safety buffer (`kSafetyBuffer`). SensorFusion stores these values into `std::optional<Object>` variables
that are then queried on `MotionPlanner::UpdateVelocity` and `MotionPlanner::UpdateTargetLane` for their values. An
empty optional means no object was found on that specific lane during a particular time cycle.

For example, `UpdateObjectToTheLeft` searches for an object with valid arclength whose _lane id distance_ to the left
(lines 62-63) is exactly one; meaning the object is in the immediately adjacent lane.

```C++
void SensorFusion::UpdateObjectToTheLeft()
{
    const Lane ego_lane{ego_state_.d};
    if (ego_lane.GetLaneId() == LaneId::kLeft)
    {
        return;
    }
    for (const auto& object : objects_)
    {
        const Lane object_lane{object.d};
        if (static_cast<int>(ego_lane.GetLaneId()) - static_cast<int>(object_lane.GetLaneId()) ==
                1 &&
            object.s > ego_state_.s && object.s - ego_state_.s <= kSafetyBuffer)
        {
            object_to_the_left_ = object;
            break;
        }
    }
}
```

As a particular precaution, before checking for vehicles in neighboring lanes these functions check whether there would
_even_ exist a neighbor lane.

## PathPlanner

The PathPlanner constructs a drivable trajectory based on the previously driven trajectory, the current ego vehicle
state, the target lane and the target velocity.

```C++
ControlTrajectory PathPlanner::GetControlTrajectory(const ControlTrajectory& previous_trajectory,
                                                    const CarState& car_state, const Lane& lane,
                                                    const double& target_velocity) const
{
    const auto& history{GetHistory(previous_trajectory, car_state)};
    const auto& reference_path{BuildReferencePath(car_state, history, lane)};
    const auto& transformed_reference_path{TransformReferencePath(reference_path, history)};
    return GenerateTrajectory(previous_trajectory, transformed_reference_path, history,
                              target_velocity);
}
```

The `GetHistory` function picks the last two points of the previouly driven trajector, if posible, or generates a past point
based on the ego vehicle's current orientation. These is a measure to ensure smoothness within consecutively generated
trajectories.

From these two points a reference path towards the target lane is drafted. The initially attempt at doing so by simply
adding some spacing (`kWaypointSpacing`) to the starting point consecutively generated trajectories that performed
_sharp-ish_ lane changes. To avoid this the distance between the first and second points is artificially smoothed
(line 118).

These points are then transformed to the ego coordinate frame using the `TransformToEgo` function on each point. The
final trajectory is then generated using the `GenerateTrajectory` function

```C++
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

        auto next_point{TransformToWorld(x, y_point, previous_yaw)};

        result.x.push_back(next_point.first + previous_x);
        result.y.push_back(next_point.second + previous_y);
    }
    return result;
}
```

Lines 85-86 insert the old trajectory in the beginning of our output vector, thus ensuring continuity. Using the
previously created reference path a spline is parametrized (lines 88-89) and the missing points to the trajectory
are calculated (lines 93-102). The step size used to generate the trajectory is dynamically computed each step based on
the target velocity

```C++
constexpr auto kMilesPerHourToMeter{2.24};
constexpr auto kSamplingRate{.02};
auto CalculateStepSize(const double& target_x, const tk::spline& spline,
                       const double& target_velocity)
{
    const auto target_y{spline(target_x)};
    const auto target_d{sqrt(target_x * target_x + target_y * target_y)};
    const auto N{target_d / (kSamplingRate * target_velocity / kMilesPerHourToMeter)};
    return target_x / N;
}
```
