#ifndef EGO_PLANNER_EMERGENCY_STOP_POLICY_H_
#define EGO_PLANNER_EMERGENCY_STOP_POLICY_H_

namespace ego_planner
{

// CINE: upstream switched the fail-safe off for good on the first depth loss, so a depth
// stream slower than the timeout ended the flight. The drone now holds its stop only while
// depth is lost and resumes once fresh depth arrives and it has stopped.
inline bool canLeaveEmergencyStop(bool fail_safe_enabled, bool depth_timed_out, double speed)
{
  constexpr double kStoppedSpeed = 0.1;

  return fail_safe_enabled && !depth_timed_out && speed < kStoppedSpeed;
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_EMERGENCY_STOP_POLICY_H_
