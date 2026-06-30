/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

#include "omni_pursuit_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

using std::hypot;
using std::min;
using std::max;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_omni_pursuit_controller
{

template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void OmniPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
}

void OmniPursuitController::cleanup() {}
void OmniPursuitController::activate() {}
void OmniPursuitController::deactivate() {}

void OmniPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/) {}

void OmniPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  progress_index_ = 0;
  // NOTE: do NOT reset the velocity ramp here. Nav2 calls setPlan on every replan
  // (~1 Hz) while the robot is moving, so zeroing prev_vx_/prev_vy_ would stutter the
  // robot to a stop each cycle. The accel limiter continues from the last command;
  // the dt > 1 s guard in computeVelocityCommands handles a genuine cold restart.
}

geometry_msgs::msg::TwistStamped OmniPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Global plan is empty, sending zero velocity");
    return cmd_vel;
  }

  // Advance progress_index_ to the closest pose from the current position,
  // searching only forward to prevent the robot from targeting poses behind itself.
  auto search_start = global_plan_.poses.begin() + progress_index_;
  auto closest_it = min_by(
    search_start, global_plan_.poses.end(),
    [&pose](const geometry_msgs::msg::PoseStamped & ps) {
      return hypot(
        pose.pose.position.x - ps.pose.position.x,
        pose.pose.position.y - ps.pose.position.y);
    });
  progress_index_ = std::distance(global_plan_.poses.begin(), closest_it);

  // From the closest point forward, find the first pose at >= lookahead_dist from the robot
  auto goal_it = std::prev(global_plan_.poses.end());
  for (auto it = closest_it; it != global_plan_.poses.end(); ++it) {
    double dist = hypot(
      pose.pose.position.x - it->pose.position.x,
      pose.pose.position.y - it->pose.position.y);
    if (dist >= lookahead_dist_) {
      goal_it = it;
      break;
    }
  }

  // Transform the lookahead pose into the robot's base frame.
  // Use time 0 to request the latest available transform, avoiding stale-timestamp failures
  // when the path was planned significantly earlier than the current control cycle.
  geometry_msgs::msg::PoseStamped goal_pose_stamped = *goal_it;
  goal_pose_stamped.header.frame_id = global_plan_.header.frame_id;
  goal_pose_stamped.header.stamp = rclcpp::Time(0);
  geometry_msgs::msg::PoseStamped goal_pose_base;
  try {
    tf_->transform(
      goal_pose_stamped, goal_pose_base, costmap_ros_->getBaseFrameID(),
      tf2::durationFromSec(transform_tolerance_.seconds()));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Failed to transform lookahead point to base frame: %s", ex.what());
    return cmd_vel;
  }

  // Scale speed down linearly as robot approaches goal
  double dist_to_goal = hypot(
    pose.pose.position.x - global_plan_.poses.back().pose.position.x,
    pose.pose.position.y - global_plan_.poses.back().pose.position.y);

  // Near the goal there is no pose >= lookahead_dist ahead, so goal_it above
  // falls back to the final plan pose and we drive straight at it. Do NOT stop
  // at lookahead_dist (that parks the robot ~lookahead_dist short of the goal
  // and the goal checker never fires). Approach-velocity scaling below slows us
  // down, and the goal checker (xy_goal_tolerance) declares success.
  double speed = desired_linear_vel_;
  if (dist_to_goal < approach_velocity_scaling_dist_) {
    speed = max(
      min_approach_linear_velocity_,
      desired_linear_vel_ * (dist_to_goal / approach_velocity_scaling_dist_));
  }

  // Compute velocity as a unit vector toward the lookahead point, scaled by speed
  double dx = goal_pose_base.pose.position.x;
  double dy = goal_pose_base.pose.position.y;
  double dist = hypot(dx, dy);

  double target_vx = 0.0;
  double target_vy = 0.0;
  if (dist > 0.001) {
    target_vx = speed * (dx / dist);
    target_vy = speed * (dy / dist);
  }

  // Acceleration limiting: clamp how much vx/vy can change this cycle. This turns an
  // instantaneous +0.2 -> -0.2 reversal (the jitter) into a ramp, and stops feeding
  // sudden motion back into odom/AMCL while localization is still settling.
  rclcpp::Time now = clock_->now();
  if (have_prev_cmd_) {
    double dt = (now - last_cmd_time_).seconds();
    if (dt > 1e-3 && dt < 1.0) {
      double max_dv = max_linear_accel_ * dt;
      target_vx = std::clamp(target_vx, prev_vx_ - max_dv, prev_vx_ + max_dv);
      target_vy = std::clamp(target_vy, prev_vy_ - max_dv, prev_vy_ + max_dv);
    }
  }
  prev_vx_ = target_vx;
  prev_vy_ = target_vy;
  last_cmd_time_ = now;
  have_prev_cmd_ = true;

  cmd_vel.twist.linear.x = target_vx;
  cmd_vel.twist.linear.y = target_vy;
  cmd_vel.twist.angular.z = 0.0;

  return cmd_vel;
}

}  // namespace nav2_omni_pursuit_controller

PLUGINLIB_EXPORT_CLASS(nav2_omni_pursuit_controller::OmniPursuitController, nav2_core::Controller)
