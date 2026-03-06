#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include "sentry_mission/health_above.hpp"
#include "sentry_mission/health_below.hpp"
#include "sentry_mission/navigate_center.hpp"
#include "sentry_mission/navigate_home.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<HealthAbove>("HealthAbove");
    factory.registerNodeType<HealthBelow>("HealthBelow");
    factory.registerNodeType<NavigateCenter>("NavigateCenter");
    factory.registerNodeType<NavigateHome>("NavigateHome");

    auto tree = factory.createTreeFromFile(
        "behavior_trees/mission_tree.xml");

    BT::StdCoutLogger logger(tree);

    while(true)
    {
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
