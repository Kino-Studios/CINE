#pragma once

#include <array>
#include <cctype>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace constants
{

struct KinematicConstraints
{
  double default_max_velocity_mps = 0.0;
  double default_max_accel_mps2 = 0.0;
  double default_speed_scale = 1.0;
  double vel_damping = 0.0;
  double acc_damping = 0.0;
  double pitch_acc_smooth_alpha = 0.0;
  double min_dt = 0.0;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
};

struct YawConstraints
{
  double yawdot_max = 0.0;
  double yaw_lpf_alpha = 0.0;
  bool use_target_yaw = false;
  bool always_face_target = false;
};

struct TargetDistance
{
  double target_distance_m = 0.0;
  double target_distance_tol_m = 0.0;
  double target_distance_gate_m = 0.0;
};

struct BsplineFilter
{
  bool use_cinematic_bspline = false;
  double cinematic_offset_m = 0.0;
  double cinematic_smooth_alpha = 0.0;
  double min_altitude_m = 0.0;
  double max_altitude_m = 0.0;
  double virtual_ceiling_m = 0.0;
};

struct FocusVelocityConstraints
{
  double focus_max_velocity_mps = 0.0;
  double focus_max_accel_mps2 = 0.0;
  double focus_speed_scale = 1.0;
};

struct FocusYawConstraints
{
  double yawdot_max_framing = 0.0;
  double yaw_lpf_alpha_framing = 0.0;
};

struct FocusDamping
{
  double focus_velocity_damping = 0.0;
  double focus_acceleration_damping = 0.0;
  double focus_yaw_damping = 0.0;
};

struct FocusDistance
{
  double desired_focus_distance_m = 0.0;
  double focus_distance_min_m = 0.0;
  double target_focus_distance_m = 0.0;
  double min_focus_distance_m = 0.0;
};

struct FocusParameters
{
  int focus_object_id = 0;
  std::array<double, 2> thirds_anchor = {0.5, 0.5};
  double thirds_strength = 1.0;
  bool thirds_use_nearest_anchor = false;
};

struct TrajServerParameters
{
  double yaw_tracking_smooth_alpha = 0.08;
  bool use_cinematic_yaw = true;
  double final_yaw_smooth_alpha_framing = 0.0;
  double yaw_dot_boost_factor = 1.0;
  double framing_exit_smooth_duration = 0.5;
  double framing_exit_acc_scale_min = 0.25;
  double framing_exit_acc_transition_s = 2.0;
  double time_forward = -1.0;
};

struct LightAvoidanceParameters
{
  double light_avoid_gain = 2.0;
  double light_avoid_blend_base = 0.08;
  double light_avoid_max_blend = 0.18;
  double light_influence_distance_m = 3.5;
  double light_avoid_transition_s = 6.0;
  double light_avoid_speed_threshold_mps = 2.5;
  double light_avoid_cooldown_after_framing_s = 0.7;
  double light_disable_near_finish_s = 0.1;
  double light_distance_blend_gain = 1.2;
  double light_rotation_damping = 0.15;
  double light_yaw_smooth_alpha = 0.35;
  double light_avoid_blend_smooth_alpha = 0.6;
  double light_avoid_weight = 0.85;
  bool use_light_avoid_yaw = true;
  bool debug_light_yaw = false;
  double debug_light_period_s = 0.2;
};

class Config
{
public:
  static const Config & instance();

  const KinematicConstraints & kinematic() const { return kinematic_; }
  const YawConstraints & yaw() const { return yaw_; }
  const TargetDistance & targetDistance() const { return target_distance_; }
  const BsplineFilter & bsplineFilter() const { return bspline_filter_; }
  const FocusVelocityConstraints & focusVelocity() const { return focus_velocity_; }
  const FocusYawConstraints & focusYaw() const { return focus_yaw_; }
  const FocusDamping & focusDamping() const { return focus_damping_; }
  const FocusDistance & focusDistance() const { return focus_distance_; }
  const FocusParameters & focusParameters() const { return focus_parameters_; }
  const TrajServerParameters & trajServer() const { return traj_server_; }
  const LightAvoidanceParameters & lightAvoidance() const { return light_avoidance_; }

private:
  Config();

  std::string loadText(const std::filesystem::path & path) const;
  std::filesystem::path findJsonRoot() const;
  std::string extractSection(const std::string & text, const std::string & key) const;
  double parseNumber(
    const std::string & section, const std::string & key, double default_value) const;
  bool parseBool(const std::string & section, const std::string & key, bool default_value) const;
  std::vector<double> parseArray(const std::string & section, const std::string & key) const;

  KinematicConstraints kinematic_;
  YawConstraints yaw_;
  TargetDistance target_distance_;
  BsplineFilter bspline_filter_;
  FocusVelocityConstraints focus_velocity_;
  FocusYawConstraints focus_yaw_;
  FocusDamping focus_damping_;
  FocusDistance focus_distance_;
  FocusParameters focus_parameters_;
  TrajServerParameters traj_server_;
  LightAvoidanceParameters light_avoidance_;
};

}  // namespace constants
