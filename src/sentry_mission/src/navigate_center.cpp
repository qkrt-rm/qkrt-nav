#include "sentry_mission/navigate_center.hpp"
#include <iostream>

NavigateCenter::NavigateCenter(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{}

BT::NodeStatus NavigateCenter::tick()
{
    // TODO: Implement actual navigation logic to center goal lol
    std::cout << "Navigate to CENTER goal" << std::endl;

    return BT::NodeStatus::SUCCESS;
}
