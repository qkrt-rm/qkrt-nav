#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class NavigateHome : public BT::SyncActionNode
{
public:
    NavigateHome(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;
};
