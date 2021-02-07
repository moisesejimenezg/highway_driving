#include "lane.h"

Lane::Lane(const double& d)
{
    if (0. <= d && d < 4.)
    {
        lane_id_ = LaneId::kLeft;
    }
    if (4. <= d && d < 8.)
    {
        lane_id_ = LaneId::kCenter;
    }
    if (8. <= d && d < 12.)
    {
        lane_id_ = LaneId::kRight;
    }
}

double Lane::GetCenterD() const { return 2. + 4. * static_cast<double>(lane_id_); }

