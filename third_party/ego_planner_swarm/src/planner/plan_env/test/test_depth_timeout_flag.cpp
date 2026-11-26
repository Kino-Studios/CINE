#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "plan_env/grid_map.h"

namespace
{
using namespace std::chrono_literals;

constexpr double kOdomDepthTimeoutSeconds = 0.3;
constexpr int kImageWidth = 64;
constexpr int kImageHeight = 48;
constexpr uint16_t kDepthMillimetres = 2000;
constexpr std::chrono::milliseconds kPublishPeriod{30};

rclcpp::NodeOptions mapNodeOptions()
{
  return rclcpp::NodeOptions().parameter_overrides({
      {"grid_map/resolution", 0.1},
      {"grid_map/map_size_x", 4.0},
      {"grid_map/map_size_y", 4.0},
      {"grid_map/map_size_z", 2.0},
      {"grid_map/local_update_range_x", 2.0},
      {"grid_map/local_update_range_y", 2.0},
      {"grid_map/local_update_range_z", 1.0},
      {"grid_map/obstacles_inflation", 0.099},
      {"grid_map/local_map_margin", 2},
      {"grid_map/ground_height", -0.01},
      {"grid_map/fx", 50.0},
      {"grid_map/fy", 50.0},
      {"grid_map/cx", 32.0},
      {"grid_map/cy", 24.0},
      {"grid_map/use_depth_filter", false},
      {"grid_map/depth_filter_tolerance", 0.15},
      {"grid_map/depth_filter_maxdist", 5.0},
      {"grid_map/depth_filter_mindist", 0.2},
      {"grid_map/depth_filter_margin", 2},
      {"grid_map/k_depth_scaling_factor", 1000.0},
      {"grid_map/skip_pixel", 2},
      {"grid_map/min_ray_length", 0.1},
      {"grid_map/max_ray_length", 4.5},
      {"grid_map/virtual_ceil_height", 1.8},
      {"grid_map/visualization_truncate_height", 1.8},
      {"grid_map/pose_type", 1},
      {"grid_map/odom_depth_timeout", kOdomDepthTimeoutSeconds},
  });
}

class DepthTimeoutFlagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map_node_ = std::make_shared<rclcpp::Node>("grid_map_under_test", mapNodeOptions());
    grid_map_ = std::make_shared<GridMap>();
    grid_map_->initMap(map_node_);

    sensor_node_ = std::make_shared<rclcpp::Node>("fake_depth_camera");
    depth_publisher_ = sensor_node_->create_publisher<sensor_msgs::msg::Image>("grid_map/depth", 10);
    pose_publisher_ = sensor_node_->create_publisher<geometry_msgs::msg::PoseStamped>("grid_map/pose", 10);

    executor_.add_node(map_node_);
    executor_.add_node(sensor_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(sensor_node_);
    executor_.remove_node(map_node_);
  }

  void publishDepthAndPose()
  {
    const rclcpp::Time stamp = sensor_node_->now();

    sensor_msgs::msg::Image depth;
    depth.header.stamp = stamp;
    depth.header.frame_id = "camera";
    depth.width = kImageWidth;
    depth.height = kImageHeight;
    depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth.step = kImageWidth * sizeof(uint16_t);
    depth.data.resize(depth.step * kImageHeight);
    uint16_t *pixels = reinterpret_cast<uint16_t *>(depth.data.data());
    std::fill(pixels, pixels + kImageWidth * kImageHeight, kDepthMillimetres);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "world";
    pose.pose.position.z = 1.0;
    pose.pose.orientation.w = 1.0;

    depth_publisher_->publish(depth);
    pose_publisher_->publish(pose);
  }

  // Publishes at a camera-like rate rather than once per spin: spin_once runs one callback,
  // so publishing every spin floods the queue with poses and starves the depth subscription.
  bool spinUntil(const std::function<bool()> &condition, bool keep_publishing,
                 std::chrono::milliseconds deadline)
  {
    const auto give_up_at = std::chrono::steady_clock::now() + deadline;
    auto next_publish_at = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < give_up_at)
    {
      if (keep_publishing && std::chrono::steady_clock::now() >= next_publish_at)
      {
        publishDepthAndPose();
        next_publish_at += kPublishPeriod;
      }
      executor_.spin_once(5ms);
      executor_.spin_some();
      if (condition())
        return true;
    }

    return false;
  }

  rclcpp::Node::SharedPtr map_node_;
  rclcpp::Node::SharedPtr sensor_node_;
  GridMap::Ptr grid_map_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(DepthTimeoutFlagTest, FlagRisesWhenDepthStopsAndClearsWhenDepthResumes)
{
  auto timed_out = [this]() { return grid_map_->getOdomDepthTimeout(); };
  auto not_timed_out = [this]() { return !grid_map_->getOdomDepthTimeout(); };

  ASSERT_TRUE(spinUntil([this]() { return grid_map_->odomValid(); }, true, 3000ms))
      << "map never received a synchronised depth image and pose";
  ASSERT_FALSE(grid_map_->getOdomDepthTimeout());

  ASSERT_TRUE(spinUntil(timed_out, false, 3000ms))
      << "flag did not rise after depth stopped for longer than the timeout";

  EXPECT_TRUE(spinUntil(not_timed_out, true, 3000ms))
      << "flag stayed set after fresh depth and pose arrived";
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
