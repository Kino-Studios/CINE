#include "constants.hpp"

#include <array>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace constants
{

namespace fs = std::filesystem;

Config::Config()
{
  const auto root = findJsonRoot();
  const auto defaults_text = loadText(root / "trajectory_constraints" / "defaults.json");
  const auto focus_text = loadText(root / "trajectory_constraints" / "focus.json");

  const auto kin_section = extractSection(defaults_text, "kinematic_constraints");
  if (!kin_section.empty()) {
    kinematic_.default_max_velocity_mps = parseNumber(kin_section, "default_max_velocity_mps", 0.0);
    kinematic_.default_max_accel_mps2 = parseNumber(kin_section, "default_max_accel_mps2", 0.0);
    kinematic_.default_speed_scale = parseNumber(kin_section, "default_speed_scale", 1.0);
    kinematic_.vel_damping = parseNumber(kin_section, "vel_damping", 0.0);
    kinematic_.pitch_acc_smooth_alpha = parseNumber(kin_section, "pitch_acc_smooth_alpha", 0.0);
    kinematic_.acc_damping = parseNumber(kin_section, "acc_damping", 0.0);
    kinematic_.min_dt = parseNumber(kin_section, "min_dt", 0.0);
    kinematic_.max_velocity = parseNumber(kin_section, "max_velocity", 0.0);
    kinematic_.max_acceleration = parseNumber(kin_section, "max_acceleration", 0.0);
  }

  const auto yaw_section = extractSection(defaults_text, "yaw_constraints");
  if (!yaw_section.empty()) {
    yaw_.yawdot_max = parseNumber(yaw_section, "yawdot_max", 0.0);
    yaw_.yaw_lpf_alpha = parseNumber(yaw_section, "yaw_lpf_alpha", 0.0);
    yaw_.use_target_yaw = parseBool(yaw_section, "use_target_yaw", false);
    yaw_.always_face_target = parseBool(yaw_section, "always_face_target", false);
  }

  const auto target_section = extractSection(defaults_text, "target_distance");
  if (!target_section.empty()) {
    target_distance_.target_distance_m = parseNumber(target_section, "target_distance_m", 0.0);
    target_distance_.target_distance_tol_m =
      parseNumber(target_section, "target_distance_tol_m", 0.0);
    target_distance_.target_distance_gate_m =
      parseNumber(target_section, "target_distance_gate_m", 0.0);
  }

  const auto bspline_section = extractSection(defaults_text, "bspline_filter");
  if (!bspline_section.empty()) {
    bspline_filter_.use_cinematic_bspline =
      parseBool(bspline_section, "use_cinematic_bspline", false);
    bspline_filter_.cinematic_offset_m = parseNumber(bspline_section, "cinematic_offset_m", 0.0);
    bspline_filter_.cinematic_smooth_alpha =
      parseNumber(bspline_section, "cinematic_smooth_alpha", 0.0);
    bspline_filter_.min_altitude_m = parseNumber(bspline_section, "min_altitude_m", 0.0);
    bspline_filter_.max_altitude_m = parseNumber(bspline_section, "max_altitude_m", 0.0);
    bspline_filter_.virtual_ceiling_m = parseNumber(bspline_section, "virtual_ceiling_m", 0.0);
  }

  const auto focus_vel_section = extractSection(focus_text, "focus_velocity_constraints");
  if (!focus_vel_section.empty()) {
    focus_velocity_.focus_max_velocity_mps =
      parseNumber(focus_vel_section, "focus_max_velocity_mps", 0.0);
    focus_velocity_.focus_max_accel_mps2 =
      parseNumber(focus_vel_section, "focus_max_accel_mps2", 0.0);
    focus_velocity_.focus_speed_scale = parseNumber(focus_vel_section, "focus_speed_scale", 1.0);
  }

  const auto focus_yaw_section = extractSection(focus_text, "focus_yaw_constraints");
  if (!focus_yaw_section.empty()) {
    focus_yaw_.yawdot_max_framing = parseNumber(focus_yaw_section, "yawdot_max_framing", 0.0);
    focus_yaw_.yaw_lpf_alpha_framing = parseNumber(focus_yaw_section, "yaw_lpf_alpha_framing", 0.0);
  }

  const auto focus_damp_section = extractSection(focus_text, "focus_damping");
  if (!focus_damp_section.empty()) {
    focus_damping_.focus_velocity_damping =
      parseNumber(focus_damp_section, "focus_velocity_damping", 0.0);
    focus_damping_.focus_acceleration_damping =
      parseNumber(focus_damp_section, "focus_acceleration_damping", 0.0);
    focus_damping_.focus_yaw_damping = parseNumber(focus_damp_section, "focus_yaw_damping", 0.0);
  }

  const auto focus_dist_section = extractSection(focus_text, "focus_distance");
  if (!focus_dist_section.empty()) {
    focus_distance_.desired_focus_distance_m =
      parseNumber(focus_dist_section, "desired_focus_distance_m", 0.0);
    focus_distance_.focus_distance_min_m =
      parseNumber(focus_dist_section, "focus_distance_min_m", 0.0);
    focus_distance_.target_focus_distance_m =
      parseNumber(focus_dist_section, "target_focus_distance_m", 0.0);
    focus_distance_.min_focus_distance_m =
      parseNumber(focus_dist_section, "min_focus_distance_m", 0.0);
  }

  const auto focus_params_section = extractSection(focus_text, "focus_parameters");
  if (!focus_params_section.empty()) {
    focus_parameters_.focus_object_id =
      static_cast<int>(parseNumber(focus_params_section, "focus_object_id", 0));
    focus_parameters_.thirds_strength = parseNumber(focus_params_section, "thirds_strength", 1.0);
    focus_parameters_.thirds_use_nearest_anchor =
      parseBool(focus_params_section, "thirds_use_nearest_anchor", false);
    const auto anchor = parseArray(focus_params_section, "thirds_anchor");
    if (anchor.size() >= 2) {
      focus_parameters_.thirds_anchor[0] = anchor[0];
      focus_parameters_.thirds_anchor[1] = anchor[1];
    }
  }

  const auto orientation_text = loadText(root / "light_avoidance" / "orientation-params.json");

  const auto traj_server_section = extractSection(orientation_text, "traj_server");
  if (!traj_server_section.empty()) {
    traj_server_.yaw_tracking_smooth_alpha = parseNumber(
      traj_server_section, "yaw_tracking_smooth_alpha", traj_server_.yaw_tracking_smooth_alpha);
    traj_server_.final_yaw_smooth_alpha_framing = parseNumber(
      traj_server_section, "final_yaw_smooth_alpha_framing",
      traj_server_.final_yaw_smooth_alpha_framing);
    traj_server_.yaw_dot_boost_factor =
      parseNumber(traj_server_section, "yaw_dot_boost_factor", traj_server_.yaw_dot_boost_factor);
    traj_server_.framing_exit_smooth_duration = parseNumber(
      traj_server_section, "framing_exit_smooth_duration",
      traj_server_.framing_exit_smooth_duration);
    traj_server_.framing_exit_acc_scale_min = parseNumber(
      traj_server_section, "framing_exit_acc_scale_min", traj_server_.framing_exit_acc_scale_min);
    traj_server_.framing_exit_acc_transition_s = parseNumber(
      traj_server_section, "framing_exit_acc_transition_s",
      traj_server_.framing_exit_acc_transition_s);
    traj_server_.use_cinematic_yaw =
      parseBool(traj_server_section, "use_cinematic_yaw", traj_server_.use_cinematic_yaw);
    traj_server_.time_forward =
      parseNumber(traj_server_section, "time_forward", traj_server_.time_forward);
  }

  const auto light_section = extractSection(orientation_text, "light_avoidance");
  if (!light_section.empty()) {
    light_avoidance_.light_avoid_gain =
      parseNumber(light_section, "light_avoid_gain", light_avoidance_.light_avoid_gain);
    light_avoidance_.light_avoid_blend_base =
      parseNumber(light_section, "light_avoid_blend_base", light_avoidance_.light_avoid_blend_base);
    light_avoidance_.light_avoid_max_blend =
      parseNumber(light_section, "light_avoid_max_blend", light_avoidance_.light_avoid_max_blend);
    light_avoidance_.light_influence_distance_m = parseNumber(
      light_section, "light_influence_distance_m", light_avoidance_.light_influence_distance_m);
    light_avoidance_.light_avoid_transition_s = parseNumber(
      light_section, "light_avoid_transition_s", light_avoidance_.light_avoid_transition_s);
    light_avoidance_.light_avoid_speed_threshold_mps = parseNumber(
      light_section, "light_avoid_speed_threshold_mps",
      light_avoidance_.light_avoid_speed_threshold_mps);
    light_avoidance_.light_avoid_cooldown_after_framing_s = parseNumber(
      light_section, "light_cooldown_after_framing_s",
      light_avoidance_.light_avoid_cooldown_after_framing_s);
    light_avoidance_.light_disable_near_finish_s = parseNumber(
      light_section, "light_disable_near_finish_s", light_avoidance_.light_disable_near_finish_s);
    light_avoidance_.light_distance_blend_gain = parseNumber(
      light_section, "light_distance_blend_gain", light_avoidance_.light_distance_blend_gain);
    light_avoidance_.light_rotation_damping =
      parseNumber(light_section, "light_rotation_damping", light_avoidance_.light_rotation_damping);
    light_avoidance_.light_yaw_smooth_alpha =
      parseNumber(light_section, "light_yaw_smooth_alpha", light_avoidance_.light_yaw_smooth_alpha);
    light_avoidance_.light_avoid_blend_smooth_alpha = parseNumber(
      light_section, "light_avoid_blend_smooth_alpha",
      light_avoidance_.light_avoid_blend_smooth_alpha);
    light_avoidance_.light_avoid_weight =
      parseNumber(light_section, "light_avoid_weight", light_avoidance_.light_avoid_weight);
    light_avoidance_.use_light_avoid_yaw =
      parseBool(light_section, "use_light_avoid_yaw", light_avoidance_.use_light_avoid_yaw);
    light_avoidance_.debug_light_yaw =
      parseBool(light_section, "debug_light_yaw", light_avoidance_.debug_light_yaw);
    light_avoidance_.debug_light_period_s =
      parseNumber(light_section, "debug_light_period_s", light_avoidance_.debug_light_period_s);
  }
}

const Config & Config::instance()
{
  static Config instance;
  return instance;
}

std::string Config::loadText(const fs::path & path) const
{
  std::ifstream reader(path);
  if (!reader) {
    throw std::runtime_error("Unable to open JSON configuration: " + path.string());
  }
  std::string contents((std::istreambuf_iterator<char>(reader)), std::istreambuf_iterator<char>());
  return contents;
}

std::filesystem::path Config::findJsonRoot() const
{
  if (const char * env = std::getenv("CINE_CONFIG_ROOT"); env != nullptr && *env != '\0') {
    const fs::path candidate(env);
    if (fs::exists(candidate)) {
      return candidate;
    }
  }

  const std::array<fs::path, 2> candidates = {
    fs::current_path() / "configs" / "json_constants",
    fs::path(__FILE__).parent_path().parent_path().parent_path() / "configs" / "json_constants"};

  for (const auto & candidate : candidates) {
    if (fs::exists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("Unable to locate JSON constants directory; set CINE_CONFIG_ROOT");
}

std::string Config::extractSection(const std::string & text, const std::string & key) const
{
  const auto token = "\"" + key + "\"";
  const auto key_pos = text.find(token);
  if (key_pos == std::string::npos) {
    return {};
  }
  const auto brace_pos = text.find('{', key_pos);
  if (brace_pos == std::string::npos) {
    return {};
  }
  int depth = 0;
  size_t idx = brace_pos;
  for (; idx < text.size(); ++idx) {
    if (text[idx] == '{') {
      ++depth;
    } else if (text[idx] == '}') {
      --depth;
      if (depth == 0) {
        return text.substr(brace_pos, idx - brace_pos + 1);
      }
    }
  }
  return {};
}

double Config::parseNumber(
  const std::string & section, const std::string & key, double default_value) const
{
  const auto token = "\"" + key + "\"";
  const auto key_pos = section.find(token);
  if (key_pos == std::string::npos) {
    return default_value;
  }
  const auto colon = section.find(':', key_pos);
  if (colon == std::string::npos) {
    return default_value;
  }
  size_t idx = colon + 1;
  while (idx < section.size() && std::isspace(static_cast<unsigned char>(section[idx]))) {
    ++idx;
  }
  size_t end = idx;
  while (end < section.size() &&
         (std::isdigit(static_cast<unsigned char>(section[end])) || section[end] == '-' ||
          section[end] == '+' || section[end] == '.' || section[end] == 'e' ||
          section[end] == 'E')) {
    ++end;
  }
  if (idx >= end) {
    return default_value;
  }
  const auto token_value = section.substr(idx, end - idx);
  try {
    return std::stod(token_value);
  } catch (...) {
    return default_value;
  }
}

bool Config::parseBool(
  const std::string & section, const std::string & key, bool default_value) const
{
  const auto token = "\"" + key + "\"";
  const auto key_pos = section.find(token);
  if (key_pos == std::string::npos) {
    return default_value;
  }
  const auto colon = section.find(':', key_pos);
  if (colon == std::string::npos) {
    return default_value;
  }
  size_t idx = colon + 1;
  while (idx < section.size() && std::isspace(static_cast<unsigned char>(section[idx]))) {
    ++idx;
  }
  if (section.compare(idx, 4, "true") == 0) {
    return true;
  }
  if (section.compare(idx, 5, "false") == 0) {
    return false;
  }
  return default_value;
}

std::vector<double> Config::parseArray(const std::string & section, const std::string & key) const
{
  std::vector<double> values;
  const auto token = "\"" + key + "\"";
  const auto key_pos = section.find(token);
  if (key_pos == std::string::npos) {
    return values;
  }
  const auto bracket = section.find('[', key_pos);
  if (bracket == std::string::npos) {
    return values;
  }
  size_t idx = bracket + 1;
  while (idx < section.size()) {
    while (idx < section.size() && std::isspace(static_cast<unsigned char>(section[idx]))) {
      ++idx;
    }
    if (idx >= section.size() || section[idx] == ']') {
      break;
    }
    size_t end = idx;
    while (end < section.size() &&
           (std::isdigit(static_cast<unsigned char>(section[end])) || section[end] == '-' ||
            section[end] == '+' || section[end] == '.' || section[end] == 'e' ||
            section[end] == 'E')) {
      ++end;
    }
    if (idx < end) {
      try {
        values.push_back(std::stod(section.substr(idx, end - idx)));
      } catch (...) {
        // ignore parse errors
      }
    }
    idx = end;
    while (idx < section.size() && section[idx] != ',' && section[idx] != ']') {
      ++idx;
    }
    if (idx < section.size() && section[idx] == ',') {
      ++idx;
    }
  }
  return values;
}

}  // namespace constants
