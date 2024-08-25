#ifndef FISHER_INFORMATION_MANAGER_HPP
#define FISHER_INFORMATION_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <unordered_map>
#include <iostream>
#include "slam_msgs/srv/get_map.hpp"
#include "frontier_exploration/planners/FrontierRoadmap.hpp"
#include "frontier_exploration/util/rosVisualizer.hpp"

const double DISTANCE3D_THRESHOLD_KF_CHANGE = 0.1; // in m
const double ANGLESUM_THRESHOLD_KF_CHANGE = 0.6; // in rad

namespace frontier_exploration
{
    // Custom hash function for std::pair<int, int>
    struct frontierPairHash {
        std::size_t operator() (const std::pair<Frontier, Frontier> &pair) const {
            auto hash1 = std::hash<size_t>{}(pair.first.getUID());
            auto hash2 = std::hash<size_t>{}(pair.second.getUID());
            return hash1 ^ (hash2 << 1); // Combine the two hashes
        }
    };
    
    class FisherInformationManager
    {
    public:
        FisherInformationManager(rclcpp::Node::SharedPtr node, std::shared_ptr<FrontierRoadMap> roadmap);

        ~FisherInformationManager();

    private:
        void callServices();
        void checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes, std::vector<int>& changedKFIds);
        void computeAndPopulateFIMs(std::shared_ptr<slam_msgs::srv::GetMap_Response> response);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr client_node_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
        std::unordered_map<int32_t, geometry_msgs::msg::PoseStamped> keyframe_poses_cache_;
        std::shared_ptr<FrontierRoadMap> roadmap_ptr_;
        std::unordered_map<std::pair<Frontier, Frontier>, float, frontierPairHash> fisher_information_map_;
        // std::shared_ptr<RosVisualizer> rosVisualizer_;
    };
}

#endif // FISHER_INFORMATION_MANAGER_HPPs