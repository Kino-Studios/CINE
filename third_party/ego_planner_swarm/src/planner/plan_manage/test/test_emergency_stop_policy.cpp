#include <gtest/gtest.h>

#include "ego_planner/emergency_stop_policy.h"

namespace
{

using ego_planner::canLeaveEmergencyStop;

TEST(EmergencyStopPolicy, LeavesOnceDepthIsBackAndDroneHasStopped)
{
  EXPECT_TRUE(canLeaveEmergencyStop(/*fail_safe_enabled=*/true, /*depth_timed_out=*/false, /*speed=*/0.05));
}

TEST(EmergencyStopPolicy, StaysWhileDepthIsTimedOut)
{
  EXPECT_FALSE(canLeaveEmergencyStop(/*fail_safe_enabled=*/true, /*depth_timed_out=*/true, /*speed=*/0.0));
}

TEST(EmergencyStopPolicy, StaysWhileDroneIsStillMoving)
{
  EXPECT_FALSE(canLeaveEmergencyStop(/*fail_safe_enabled=*/true, /*depth_timed_out=*/false, /*speed=*/0.2));
}

TEST(EmergencyStopPolicy, StaysWhenFailSafeIsDisabled)
{
  EXPECT_FALSE(canLeaveEmergencyStop(/*fail_safe_enabled=*/false, /*depth_timed_out=*/false, /*speed=*/0.0));
}

}  // namespace
