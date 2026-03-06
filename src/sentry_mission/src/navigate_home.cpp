#include "sentry_mission/navigate_home.hpp"
#include <iostream>

NavigateHome::NavigateHome(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{}

BT::NodeStatus NavigateHome::tick()
{
    // TODO: Implement actual navigation logic to home goal lol
    std::cout << "Navigate HOME to heal" << std::endl;

    return BT::NodeStatus::SUCCESS;
}
