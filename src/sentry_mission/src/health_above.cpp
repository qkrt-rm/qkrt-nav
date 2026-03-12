#include "sentry_mission/health_above.hpp"

HealthAbove::HealthAbove(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
    health = 20;  // TODO: Assuming full health for now, need to get actual health from somewhere
}

BT::NodeStatus HealthAbove::tick()
{
    int threshold;
    getInput("threshold", threshold);

    if(health > threshold)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}
