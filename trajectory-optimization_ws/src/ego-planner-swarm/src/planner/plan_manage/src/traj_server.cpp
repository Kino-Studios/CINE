#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/msg/point_stamped.hpp"
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

static bool has_target_ = false;
static Eigen::Vector3d target_world_(Eigen::Vector3d::Zero());
static rclcpp::Time target_stamp_;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
std::vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

double default_speed_scale_ = 0.8;
double focus_speed_scale_ = 0.4;
double default_max_vel_ = 1.2;
double default_max_acc_ = 1.6;
double focus_max_vel_ = 0.9;
double focus_max_acc_ = 1.4;
double target_dist_m_ = 1.0;
double target_dist_tol_m_ = 0.05;
double target_dist_gate_m_ = 1.0;
double yawdot_max_ = 2.5;
double yawdot_max_framing_ = 1.2;
double yaw_lpf_alpha_ = 0.5;
double yaw_lpf_alpha_framing_ = 0.8;
bool use_target_yaw_ = true;
bool use_cinematic_ = true;
bool framing_active_ = false;
double vel_damping_ = 0.15;
double acc_damping_ = 0.05;

Eigen::Vector3d last_vel_(Eigen::Vector3d::Zero());
Eigen::Vector3d last_acc_(Eigen::Vector3d::Zero());

void targetWorldCallback(const geometry_msgs::msg::PointStamped::ConstPtr msg)
{
  target_world_(0) = msg->point.x;
  target_world_(1) = msg->point.y;
  target_world_(2) = msg->point.z;
  target_stamp_ = msg->header.stamp;
  has_target_ = true;
}

void targetDistanceCallback(const std_msgs::msg::Float32::ConstPtr msg)
{
  if (!msg)
    return;
  double dist = static_cast<double>(msg->data);
  if (std::isfinite(dist) && dist > 0.0)
  {
    target_dist_m_ = dist;
  }
}

void framingActiveCallback(const std_msgs::msg::Bool::ConstPtr msg)
{
  if (!msg)
    return;
  framing_active_ = msg->data;
}

void handleBsplineMsg(traj_utils::msg::Bspline::ConstPtr msg)
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

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  handleBsplineMsg(msg);
}

void cinematicBsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
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
  double speed_scale = default_speed_scale_;
  bool focus_active = false;
  if (has_target_ && framing_active_)
  {
    double t_probe = std::min(traj_duration_, t_cur * default_speed_scale_);
    Eigen::Vector3d pos_probe = traj_[0].evaluateDeBoorT(t_probe);
    double dist = (pos_probe - target_world_).norm();
    double gate = std::max(target_dist_gate_m_, target_dist_tol_m_);
    focus_active = fabs(dist - target_dist_m_) <= gate;
  }
  if (focus_active)
  {
    speed_scale = focus_speed_scale_;
  }

  double t_query = t_cur * speed_scale;
  if (t_query < 0.0)
    t_query = 0.0;

  constexpr double PI = 3.141592653589793;
  if (t_query < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_query);
    vel = traj_[1].evaluateDeBoorT(t_query) * speed_scale;
    acc = traj_[2].evaluateDeBoorT(t_query) * speed_scale * speed_scale;

    if (focus_active && has_target_)
    {
      Eigen::Vector3d r = pos - target_world_;
      double r_norm = r.norm();
      if (r_norm > 1e-3)
      {
        Eigen::Vector3d r_hat = r / r_norm;
        pos = target_world_ + r_hat * target_dist_m_;
        double v_rad = vel.dot(r_hat);
        vel = vel - v_rad * r_hat;
        double v_sq = vel.squaredNorm();
        acc = acc - acc.dot(r_hat) * r_hat;
        acc = acc - (v_sq / std::max(target_dist_m_, 1e-3)) * r_hat;
      }
    }

    /*** calculate yaw ***/
    if (use_target_yaw_ && focus_active)
    {
      double yaw_target = atan2(target_world_(1) - pos(1), target_world_(0) - pos(0));
      double max_change = yawdot_max_framing_ * (time_now - time_last).seconds();
      double delta = yaw_target - last_yaw_;
      if (delta > PI) delta -= 2 * PI;
      if (delta < -PI) delta += 2 * PI;
      if (fabs(delta) > max_change)
        delta = (delta > 0 ? 1.0 : -1.0) * max_change;
      double yaw = last_yaw_ + delta;
      double alpha = std::clamp(yaw_lpf_alpha_framing_, 0.0, 1.0);
      yaw = alpha * last_yaw_ + (1.0 - alpha) * yaw;
      double yawdot = (time_now - time_last).seconds() > 1e-3 ? delta / (time_now - time_last).seconds() : 0.0;
      yaw_yawdot.first = yaw;
      yaw_yawdot.second = yawdot;
      last_yaw_ = yaw;
      last_yaw_dot_ = yawdot;
    }
    else
    {
      yaw_yawdot = calculate_yaw(t_query, pos, time_now, time_last);
    }
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_query + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_query >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    std::cout << "[Traj server]: invalid time." << std::endl;
  }
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

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  node->declare_parameter("traj_server/use_cinematic_bspline", true);
  use_cinematic_ = node->get_parameter("traj_server/use_cinematic_bspline").as_bool();
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
  node->declare_parameter("traj_server/vel_damping", vel_damping_);
  node->declare_parameter("traj_server/acc_damping", acc_damping_);

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
  vel_damping_ = node->get_parameter("traj_server/vel_damping").as_double();
  acc_damping_ = node->get_parameter("traj_server/acc_damping").as_double();

  // Store subscription to keep it alive - CRITICAL: must not go out of scope!
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_subscription;
  
  // Subscribe to appropriate B-spline topic based on parameter
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

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
