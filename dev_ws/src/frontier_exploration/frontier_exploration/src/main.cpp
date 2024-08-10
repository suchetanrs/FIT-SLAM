#include "rclcpp/rclcpp.hpp"
#include <frontier_exploration/ExplorationBT.hpp>

void threadFunction(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ptr)
{
    ptr->spin();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_exploration_node");
    auto serverObject = std::make_shared<frontier_exploration::FrontierExplorationServer>(node);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread t1(threadFunction, executor);
    t1.detach();
    serverObject->makeBTNodes();
    // serverObject->run();

    rclcpp::shutdown();
    return 0;
}