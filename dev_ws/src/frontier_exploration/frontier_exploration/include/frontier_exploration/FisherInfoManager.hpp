#ifndef FISHER_INFORMATION_MANAGER_HPP
#define FISHER_INFORMATION_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "slam_msgs/srv/get_map.hpp"
#include <chrono>
#include <vector>
#include <unordered_map>

class FisherInformationManager
{
public:
    FisherInformationManager(rclcpp::Node::SharedPtr node);

    ~FisherInformationManager();

private:
    void callService();
    void checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int32_t, slam_msgs::msg::KeyFrame> previous_keyframes_;
};

#endif // FISHER_INFORMATION_MANAGER_HPPs