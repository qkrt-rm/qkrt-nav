/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#ifndef NAV2_OMNI_PURSUIT_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_
#define NAV2_OMNI_PURSUIT_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_omni_pursuit_controller
{

class OmniPursuitController : public nav2_core::Controller
{
public:
  OmniPursuitController() = default;
  ~OmniPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("OmniPursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  double approach_velocity_scaling_dist_;
  double min_approach_linear_velocity_;
  double max_linear_accel_;
  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;
  size_t progress_index_{0};

  // Output acceleration limiting: ramp vx/vy between cycles instead of allowing
  // instantaneous reversals (the +0.2 -> -0.2 jitter when the lookahead flips).
  double prev_vx_{0.0};
  double prev_vy_{0.0};
  rclcpp::Time last_cmd_time_;
  bool have_prev_cmd_{false};
};

}  // namespace nav2_omni_pursuit_controller

#endif  // NAV2_OMNI_PURSUIT_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_
