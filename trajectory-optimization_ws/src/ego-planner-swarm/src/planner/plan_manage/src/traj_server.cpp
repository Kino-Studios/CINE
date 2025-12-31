#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr focus_vector_pub;
rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr light_vector_sub;

static Eigen::Vector3d light_vector_(Eigen::Vector3d::Zero());
static bool have_light_vector_ = false;
static double light_distance_ = 0.0;
static double light_avoid_gain_ = 0.2;
static double light_avoid_blend_base_ = 0.05;
static double light_avoid_max_blend_ = 0.25;
static double light_influence_distance_m_ = 6.0;
static double light_avoid_transition_s_ = 6.0;
static double light_avoid_speed_threshold_ = 0.4;
static bool use_light_avoid_yaw_ = true;
static bool debug_light_yaw_ = false;
static double debug_light_period_s_ = 0.5;
static rclcpp::Time last_light_debug_time_(0L, RCL_ROS_TIME);

static bool has_target_ = false;
static Eigen::Vector3d target_world_(Eigen::Vector3d::Zero());
static rclcpp::Time target_stamp_;
static double light_avoid_cooldown_after_framing_s = 1.0;
static rclcpp::Time last_framing_toggle_time_(0L, RCL_ROS_TIME);

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

static double last_speed_log_time_sec = -1.0;

static const rclcpp::Logger traj_logger = rclcpp::get_logger("traj_server");

bool receive_traj_ = false;
std::vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

double default_speed_scale_ = 0.9;
double focus_speed_scale_ = 0.7;
double default_max_vel_ = 2.2;
double default_max_acc_ = 4.0;
double focus_max_vel_ = 0.8;
double focus_max_acc_ = 3.0;
double target_dist_m_ = 1.0;
double target_dist_tol_m_ = 0.05;
double target_dist_gate_m_ = 1.0;
double yawdot_max_ = 1.25;
double yawdot_max_framing_ = 0.6;
double yaw_lpf_alpha_ = 0.5;
double yaw_lpf_alpha_framing_ = 0.8;
bool use_target_yaw_ = true;
bool always_face_target_ = true;
bool use_cinematic_ = true;
bool framing_active_ = false;
double vel_damping_ = 0.03;
double acc_damping_ = 0.05;

Eigen::Vector3d last_vel_(Eigen::Vector3d::Zero());
Eigen::Vector3d last_acc_(Eigen::Vector3d::Zero());

void targetWorldCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  target_world_(0) = msg->point.x;
  target_world_(1) = msg->point.y;
  target_world_(2) = msg->point.z;
  target_stamp_ = msg->header.stamp;
  has_target_ = true;
}

void targetDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg)
    return;
  double dist = static_cast<double>(msg->data);
  if (std::isfinite(dist) && dist > 0.0)
  {
    target_dist_m_ = dist;
  }
}

void framingActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg)
    return;
  framing_active_ = msg->data;
  last_framing_toggle_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
}

void lightVectorCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  if (!msg)
    return;
  light_vector_(0) = msg->vector.x;
  light_vector_(1) = msg->vector.y;
  light_vector_(2) = msg->vector.z;
  light_distance_ = light_vector_.norm();
  have_light_vector_ = light_distance_ > 1e-6;
}

void handleBsplineMsg(traj_utils::msg::Bspline::SharedPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  last_vel_.setZero();
  last_acc_.setZero();
}

void bsplineCallback(traj_utils::msg::Bspline::SharedPtr msg)
{
  handleBsplineMsg(msg);
}

void cinematicBsplineCallback(traj_utils::msg::Bspline::SharedPtr msg)
{
  handleBsplineMsg(msg);
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
  {
    double alpha = std::clamp(yaw_lpf_alpha_, 0.0, 1.0);
    yaw = alpha * last_yaw_ + (1.0 - alpha) * yaw;
  }
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void updateYawTowardsTarget(const Eigen::Vector3d &pos, bool focus_active, double dt, std::pair<double, double> &yaw_yawdot)
{
  double yaw_limit = focus_active ? yawdot_max_framing_ : yawdot_max_;
  double safe_dt = std::max(0.0, dt);
  double max_change = yaw_limit * safe_dt;
  Eigen::Vector3d dir = target_world_ - pos;
  double yaw_target = last_yaw_;
  if (dir.norm() > 1e-3)
  {
    yaw_target = atan2(dir(1), dir(0));
  }
  double delta = yaw_target - last_yaw_;
  if (delta > PI)
  {
    delta -= 2 * PI;
  }
  else if (delta < -PI)
  {
    delta += 2 * PI;
  }
  if (max_change > 0.0 && std::fabs(delta) > max_change)
  {
    delta = (delta > 0 ? max_change : -max_change);
  }
  double yaw = last_yaw_ + delta;
  double alpha = std::clamp(focus_active ? yaw_lpf_alpha_framing_ : yaw_lpf_alpha_, 0.0, 1.0);
  yaw = alpha * last_yaw_ + (1.0 - alpha) * yaw;
  double yawdot = safe_dt > 1e-3 ? delta / safe_dt : 0.0;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;
}

double blendAngles(double base, double target, double weight)
{
  if (weight <= 0.0)
  {
    return base;
  }
  double diff = std::atan2(std::sin(target - base), std::cos(target - base));
  double mixed = base + diff * weight;
  return std::atan2(std::sin(mixed), std::cos(mixed));
}

void applyLightYawAvoidance(double dt, const rclcpp::Time &time_now, std::pair<double, double> &yaw_yawdot, double current_speed)
{
  if (!use_light_avoid_yaw_ || framing_active_ || !have_light_vector_ || light_influence_distance_m_ <= 0.0)
  {
    return;
  }
  if (light_avoid_speed_threshold_ > 0.0 && current_speed > light_avoid_speed_threshold_)
  {
    return;
  }
  double since_framing = (time_now - last_framing_toggle_time_).seconds();
  if (!framing_active_ && since_framing >= 0.0 && since_framing < light_avoid_cooldown_after_framing_s)
  {
    return;
  }

  double dist = light_distance_;
  if (dist <= 0.0)
  {
    return;
  }

  double weight = (light_influence_distance_m_ - dist) / light_influence_distance_m_;
  weight = std::clamp(weight, 0.0, 1.0);
  if (weight <= 0.0)
  {
    return;
  }

  double avoidance_yaw = std::atan2(-light_vector_(1), -light_vector_(0));

  double blend = light_avoid_blend_base_ + light_avoid_gain_ * std::pow(weight, 2);
  double blend_limit = std::max(light_avoid_blend_base_, light_avoid_max_blend_);
  blend = std::clamp(blend, light_avoid_blend_base_, blend_limit);

  double ramp = 1.0;

  if (since_framing >= light_avoid_cooldown_after_framing_s)
  {
    double ramp_time = since_framing - light_avoid_cooldown_after_framing_s;
    if (light_avoid_transition_s_ > 0.0)
    {
      ramp = std::clamp(ramp_time / light_avoid_transition_s_, 0.0, 1.0);
    }
  }

  blend *= (1.0 - 0.6 * std::pow(1.0 - ramp, 3));
  double original_yaw = yaw_yawdot.first;
  double target_yaw = blendAngles(original_yaw, avoidance_yaw, blend);
  double delta = std::atan2(std::sin(target_yaw - original_yaw), std::cos(target_yaw - original_yaw));
  double safe_dt = std::max(0.0, dt);
  double max_change = yawdot_max_ * safe_dt;
  if (max_change > 0.0 && std::fabs(delta) > max_change)
  {
    delta = (delta > 0 ? max_change : -max_change);
  }

  double new_yaw = std::atan2(std::sin(original_yaw + delta), std::cos(original_yaw + delta));
  double yawdot = safe_dt > 1e-6 ? delta / safe_dt : 0.0;
  yaw_yawdot.first = new_yaw;
  yaw_yawdot.second = yawdot;

  if (debug_light_yaw_ &&
      (time_now - last_light_debug_time_).seconds() >= debug_light_period_s_)
  {
    RCLCPP_INFO(
        traj_logger,
        "[Traj server]: light avoidance yaw blend=%.2f weight=%.2f distance=%.2f",
        blend,
        weight,
        dist);
    last_light_debug_time_ = time_now;
  }
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // 统一时间源
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  bool focus_active = false;
  if (has_target_ && framing_active_)
  {
    Eigen::Vector3d pos_current = traj_[0].evaluateDeBoorT(std::min(traj_duration_, t_cur));
    double dist = (pos_current - target_world_).norm();
    double gate = std::max(target_dist_gate_m_, target_dist_tol_m_);
    focus_active = fabs(dist - target_dist_m_) <= gate;
  }

  bool follow_target = has_target_ && framing_active_ && (always_face_target_ || (use_target_yaw_ && focus_active));
  double dt = std::max(0.0, (time_now - time_last).seconds());
  bool have_position = false;

  double t_query = std::max(0.0, std::min(traj_duration_, t_cur));
  double raw_speed = 0.0;
  double pre_focus_speed = 0.0;
  double post_focus_speed = 0.0;
  bool valid_plan = false;

  constexpr double PI = 3.141592653589793;
  double speed_scale = focus_active ? focus_speed_scale_ : default_speed_scale_;
  if (t_query < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_query);
    Eigen::Vector3d raw_vel = traj_[1].evaluateDeBoorT(t_query);
    raw_speed = raw_vel.norm();
    vel = raw_vel * speed_scale;
    pre_focus_speed = vel.norm();
    acc = traj_[2].evaluateDeBoorT(t_query) * speed_scale * speed_scale;

    if (focus_active && has_target_)
    {
      Eigen::Vector3d r = pos - target_world_;
      double r_norm = r.norm();
      if (r_norm > 1e-3)
      {
        Eigen::Vector3d r_hat = r / r_norm;
        double v_rad = vel.dot(r_hat);
        double dist_error = r_norm - target_dist_m_;
        if (fabs(dist_error) > target_dist_tol_m_)
        {
          double damp_factor = 0.5;
          vel = vel - (1.0 - damp_factor) * v_rad * r_hat;
        }
        acc = acc - acc.dot(r_hat) * r_hat;
      }
    }
    post_focus_speed = vel.norm();


    /*** calculate yaw ***/
    if (follow_target)
    {
      updateYawTowardsTarget(pos, focus_active, dt, yaw_yawdot);
    }
    else
    {
      yaw_yawdot = calculate_yaw(t_query, pos, time_now, time_last);
    }
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_query + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
    valid_plan = true;
    have_position = true;
  }
  else if (t_query >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    have_position = true;
    if (follow_target)
    {
      updateYawTowardsTarget(pos, focus_active, dt, yaw_yawdot);
    }
    else
    {
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0;
    }

    pos_f = pos;
  }
  else
  {
    double start_sec = start_time_.seconds();
    double now_sec = time_now.seconds();
    RCLCPP_WARN(
        traj_logger,
        "[Traj server]: invalid time (t_cur=%.3f, start=%.3f, now=%.3f, duration=%.3f); waiting for ROS time to catch up",
        t_cur,
        start_sec,
        now_sec,
        traj_duration_);
  }
  double current_speed = vel.norm();
  applyLightYawAvoidance(dt, time_now, yaw_yawdot, current_speed);
  time_last = time_now;

  double max_vel = focus_active ? focus_max_vel_ : default_max_vel_;
  double max_acc = focus_active ? focus_max_acc_ : default_max_acc_;

  double vel_damp = std::clamp(vel_damping_, 0.0, 1.0);
  double acc_damp = std::clamp(acc_damping_, 0.0, 1.0);
  vel = vel * (1.0 - vel_damp) + last_vel_ * vel_damp;
  acc = acc * (1.0 - acc_damp) + last_acc_ * acc_damp;

  double vel_norm = vel.norm();
  if (vel_norm > max_vel && vel_norm > 1e-6)
  {
    vel = vel / vel_norm * max_vel;
  }
  double acc_norm = acc.norm();
  if (acc_norm > max_acc && acc_norm > 1e-6)
  {
    acc = acc / acc_norm * max_acc;
  }
  double filtered_speed = vel.norm();

  if (valid_plan)
  {
    double now_sec = time_now.seconds();
    if (last_speed_log_time_sec < 0 || now_sec - last_speed_log_time_sec >= 1.0)
    {
      last_speed_log_time_sec = now_sec;
      RCLCPP_INFO(
          traj_logger,
          "[Traj server]: speed raw=%.3f pre_focus=%.3f post_focus=%.3f filtered=%.3f focus=%s speed_scale=%.2f max_vel=%.2f",
          raw_speed,
          pre_focus_speed,
          post_focus_speed,
          filtered_speed,
          focus_active ? "true" : "false",
          speed_scale,
          max_vel);
    }
  }

  last_vel_ = vel;
  last_acc_ = acc;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  if (has_target_ && have_position && framing_active_ && focus_vector_pub)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = time_now;
    marker.header.frame_id = "world";
    marker.ns = "focus_vector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.1;
    marker.color.g = 0.75;
    marker.color.b = 0.75;
    marker.color.a = 0.9;
    marker.points.resize(2);
    marker.points[0].x = pos(0);
    marker.points[0].y = pos(1);
    marker.points[0].z = pos(2);
    marker.points[1].x = target_world_(0);
    marker.points[1].y = target_world_(1);
    marker.points[1].z = target_world_(2);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    focus_vector_pub->publish(marker);
  }

pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  node->declare_parameter("traj_server/use_cinematic_bspline", true);
  node->declare_parameter("traj_server/default_speed_scale", default_speed_scale_);
  node->declare_parameter("traj_server/focus_speed_scale", focus_speed_scale_);
  node->declare_parameter("traj_server/default_max_vel", default_max_vel_);
  node->declare_parameter("traj_server/default_max_acc", default_max_acc_);
  node->declare_parameter("traj_server/focus_max_vel", focus_max_vel_);
  node->declare_parameter("traj_server/focus_max_acc", focus_max_acc_);
  node->declare_parameter("traj_server/target_distance_m", target_dist_m_);
  node->declare_parameter("traj_server/target_distance_tol_m", target_dist_tol_m_);
  node->declare_parameter("traj_server/target_distance_gate_m", target_dist_gate_m_);
  node->declare_parameter("traj_server/yawdot_max", yawdot_max_);
  node->declare_parameter("traj_server/yawdot_max_framing", yawdot_max_framing_);
  node->declare_parameter("traj_server/yaw_lpf_alpha", yaw_lpf_alpha_);
  node->declare_parameter("traj_server/yaw_lpf_alpha_framing", yaw_lpf_alpha_framing_);
  node->declare_parameter("traj_server/use_target_yaw", use_target_yaw_);
  node->declare_parameter("traj_server/always_face_target", always_face_target_);
  node->declare_parameter("traj_server/vel_damping", vel_damping_);
  node->declare_parameter("traj_server/acc_damping", acc_damping_);
  node->declare_parameter("traj_server/light_avoid_gain", light_avoid_gain_);
  node->declare_parameter("traj_server/light_influence_distance_m", light_influence_distance_m_);
  node->declare_parameter("traj_server/use_light_avoid_yaw", use_light_avoid_yaw_);
  node->declare_parameter("traj_server/debug_light_yaw", debug_light_yaw_);
  node->declare_parameter("traj_server/debug_light_period_s", debug_light_period_s_);
  node->declare_parameter("traj_server/light_avoid_blend_base", light_avoid_blend_base_);
  node->declare_parameter("traj_server/light_avoid_max_blend", light_avoid_max_blend_);
  node->declare_parameter("traj_server/light_avoid_speed_threshold", light_avoid_speed_threshold_);
  node->declare_parameter("traj_server/light_avoid_transition_s", light_avoid_transition_s_);
  node->declare_parameter("traj_server/light_avoid_cooldown_after_framing_s", light_avoid_cooldown_after_framing_s);
  node->declare_parameter("traj_server/time_forward", -1.0);

  use_cinematic_ = node->get_parameter("traj_server/use_cinematic_bspline").as_bool();
  default_speed_scale_ = node->get_parameter("traj_server/default_speed_scale").as_double();
  focus_speed_scale_ = node->get_parameter("traj_server/focus_speed_scale").as_double();
  default_max_vel_ = node->get_parameter("traj_server/default_max_vel").as_double();
  default_max_acc_ = node->get_parameter("traj_server/default_max_acc").as_double();
  focus_max_vel_ = node->get_parameter("traj_server/focus_max_vel").as_double();
  focus_max_acc_ = node->get_parameter("traj_server/focus_max_acc").as_double();
  target_dist_m_ = node->get_parameter("traj_server/target_distance_m").as_double();
  target_dist_tol_m_ = node->get_parameter("traj_server/target_distance_tol_m").as_double();
  target_dist_gate_m_ = node->get_parameter("traj_server/target_distance_gate_m").as_double();
  yawdot_max_ = node->get_parameter("traj_server/yawdot_max").as_double();
  yawdot_max_framing_ = node->get_parameter("traj_server/yawdot_max_framing").as_double();
  yaw_lpf_alpha_ = node->get_parameter("traj_server/yaw_lpf_alpha").as_double();
  yaw_lpf_alpha_framing_ = node->get_parameter("traj_server/yaw_lpf_alpha_framing").as_double();
  use_target_yaw_ = node->get_parameter("traj_server/use_target_yaw").as_bool();
  always_face_target_ = node->get_parameter("traj_server/always_face_target").as_bool();
  vel_damping_ = node->get_parameter("traj_server/vel_damping").as_double();
  acc_damping_ = node->get_parameter("traj_server/acc_damping").as_double();
  light_avoid_gain_ = node->get_parameter("traj_server/light_avoid_gain").as_double();
  light_influence_distance_m_ = node->get_parameter("traj_server/light_influence_distance_m").as_double();
  use_light_avoid_yaw_ = node->get_parameter("traj_server/use_light_avoid_yaw").as_bool();
  debug_light_yaw_ = node->get_parameter("traj_server/debug_light_yaw").as_bool();
  debug_light_period_s_ = node->get_parameter("traj_server/debug_light_period_s").as_double();
  light_avoid_blend_base_ = node->get_parameter("traj_server/light_avoid_blend_base").as_double();
  light_avoid_max_blend_ = node->get_parameter("traj_server/light_avoid_max_blend").as_double();
  light_avoid_speed_threshold_ = node->get_parameter("traj_server/light_avoid_speed_threshold").as_double();
  light_avoid_transition_s_ = node->get_parameter("traj_server/light_avoid_transition_s").as_double();
  light_avoid_cooldown_after_framing_s = node->get_parameter("traj_server/light_avoid_cooldown_after_framing_s").as_double();
  node->get_parameter("traj_server/time_forward", time_forward_);

  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_subscription;
  if (use_cinematic_) {
    RCLCPP_INFO(node->get_logger(), "Using CINEMATIC B-spline trajectory (planning/cinematic_bspline)");
    bspline_subscription = node->create_subscription<traj_utils::msg::Bspline>(
        "planning/cinematic_bspline",
        10,
        cinematicBsplineCallback);
  } else {
    RCLCPP_INFO(node->get_logger(), "Using BASELINE B-spline trajectory (planning/bspline)");
    bspline_subscription = node->create_subscription<traj_utils::msg::Bspline>(
        "planning/bspline",
        10,
        bsplineCallback);
  }

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  focus_vector_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      "/cinematography/focus_vector",
      10);

  auto target_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "/cinematography/target_world",
      10,
      targetWorldCallback);
  (void)target_sub;

  auto target_dist_sub = node->create_subscription<std_msgs::msg::Float32>(
      "/cinematography/target_distance_m",
      10,
      targetDistanceCallback);
  (void)target_dist_sub;

  auto framing_sub = node->create_subscription<std_msgs::msg::Bool>(
      "/cinematography/framing_active",
      10,
      framingActiveCallback);
  (void)framing_sub;

  light_vector_sub = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/cinematography/light_vector",
      10,
      lightVectorCallback);
  (void)light_vector_sub;

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];
  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
