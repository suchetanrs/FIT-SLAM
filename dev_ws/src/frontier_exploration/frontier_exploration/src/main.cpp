#include "rclcpp/rclcpp.hpp"
#include <frontier_exploration/explore_server.hpp>

void threadFunction(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ptr)
{
    ptr->spin();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto serverObject = std::make_shared<frontier_exploration::FrontierExplorationServer>();

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(serverObject);
    std::thread t1(threadFunction, executor);
    t1.detach();
    serverObject->run();

    rclcpp::shutdown();
    return 0;
}