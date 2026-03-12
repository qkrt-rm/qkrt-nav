#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include "sentry_mission/health_above.hpp"
#include "sentry_mission/health_below.hpp"
#include "sentry_mission/navigate_center.hpp"
#include "sentry_mission/navigate_home.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<HealthAbove>("HealthAbove");
    factory.registerNodeType<HealthBelow>("HealthBelow");
    factory.registerNodeType<NavigateCenter>("NavigateCenter");
    factory.registerNodeType<NavigateHome>("NavigateHome");

    std::string pkg_path = ament_index_cpp::get_package_share_directory("sentry_mission");
    std::string tree_path = pkg_path + "/behavior_trees/mission_tree.xml";

    auto tree = factory.createTreeFromFile(tree_path);
    BT::StdCoutLogger logger(tree);

    while(rclcpp::ok())
    {
        // TODO: Log status when there is changes instead of printing to terminal (we don't to flood)
        std::cout << "[BT] Ticking root" << std::endl;
        auto status = tree.tickRoot();
        std::cout << "[BT] Root returned: " << BT::toStr(status) << std::endl;

        // TODO: Increase tick rate for better response for our use, not sure what we want yet
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    rclcpp::shutdown();
}