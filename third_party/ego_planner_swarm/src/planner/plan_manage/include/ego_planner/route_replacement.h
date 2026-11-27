#ifndef EGO_PLANNER_ROUTE_REPLACEMENT_H_
#define EGO_PLANNER_ROUTE_REPLACEMENT_H_

#include <vector>

#include <Eigen/Eigen>

namespace ego_planner
{

// CINE: upstream appended every goal that arrived in flight to the end of its queue, so a
// client that re-sent its goals after changing its route left the old goals queued ahead of
// the new ones, and the drone flew them first. A route message now replaces the queue, and
// the planner replans only when the new route does not start at the goal it is flying to.
inline bool routeStartsAtCurrentTarget(const std::vector<Eigen::Vector3d> &queue, int current_index,
                                       const Eigen::Vector3d &route_start)
{
  constexpr double kSameGoalM = 1e-3;

  if (current_index < 0 || current_index >= static_cast<int>(queue.size()))
    return false;

  return (queue[current_index] - route_start).norm() < kSameGoalM;
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_ROUTE_REPLACEMENT_H_
