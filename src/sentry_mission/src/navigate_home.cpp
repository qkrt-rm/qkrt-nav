#include "sentry_mission/navigate_home.hpp"

NavigateHome::NavigateHome(
    const std::string& name,
    const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("navigate_home_bt");
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
}

BT::NodeStatus NavigateHome::tick()
{
    if(!client_->wait_for_action_server(std::chrono::seconds(2)))
        return BT::NodeStatus::FAILURE;

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();

    goal_msg.pose.pose.position.x = -2.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto future_goal_handle = client_->async_send_goal(goal_msg);

    rclcpp::spin_until_future_complete(node_, future_goal_handle);

    return BT::NodeStatus::SUCCESS;
}