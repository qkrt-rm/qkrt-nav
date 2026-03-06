#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

class HealthBelow : public BT::ConditionNode
{
public:
    HealthBelow(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("threshold") };
    }

    BT::NodeStatus tick() override;

private:
    int health;
};
