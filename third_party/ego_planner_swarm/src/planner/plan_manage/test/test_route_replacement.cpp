#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Eigen>

#include "ego_planner/route_replacement.h"

namespace
{

using ego_planner::routeStartsAtCurrentTarget;

const std::vector<Eigen::Vector3d> kQueue = {
    Eigen::Vector3d(0.0, 0.0, 2.0),
    Eigen::Vector3d(1.0, 0.0, 2.0),
    Eigen::Vector3d(2.0, 0.0, 2.0),
};

TEST(RouteReplacement, KeepsFlyingWhenTheRouteStartsAtTheCurrentTarget)
{
  EXPECT_TRUE(routeStartsAtCurrentTarget(kQueue, /*current_index=*/1, Eigen::Vector3d(1.0, 0.0, 2.0)));
}

TEST(RouteReplacement, ReplansWhenTheRouteStartsElsewhere)
{
  EXPECT_FALSE(routeStartsAtCurrentTarget(kQueue, /*current_index=*/1, Eigen::Vector3d(1.0, 0.5, 2.0)));
}

TEST(RouteReplacement, ReplansWhenTheRouteStartsAtAnAlreadyPassedWaypoint)
{
  EXPECT_FALSE(routeStartsAtCurrentTarget(kQueue, /*current_index=*/1, Eigen::Vector3d(0.0, 0.0, 2.0)));
}

TEST(RouteReplacement, ReplansWhenNothingIsQueued)
{
  EXPECT_FALSE(routeStartsAtCurrentTarget({}, /*current_index=*/0, Eigen::Vector3d(0.0, 0.0, 2.0)));
}

TEST(RouteReplacement, ReplansWhenTheCurrentIndexIsPastTheQueue)
{
  EXPECT_FALSE(routeStartsAtCurrentTarget(kQueue, /*current_index=*/3, Eigen::Vector3d(2.0, 0.0, 2.0)));
}

}  // namespace
