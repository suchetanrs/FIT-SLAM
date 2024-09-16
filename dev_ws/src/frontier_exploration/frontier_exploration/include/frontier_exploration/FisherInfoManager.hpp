#ifndef FISHER_INFORMATION_MANAGER_HPP
#define FISHER_INFORMATION_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <unordered_map>
#include <iostream>
#include "slam_msgs/srv/get_landmarks_in_view.hpp"
#include "frontier_exploration/planners/FrontierRoadmap.hpp"
#include "frontier_exploration/util/rosVisualizer.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

const double DISTANCE3D_THRESHOLD_KF_CHANGE = 0.1; // in m
const double ANGLESUM_THRESHOLD_KF_CHANGE = 0.6; // in rad

struct LookupKey {
    std::array<float, 3> components;

    bool operator==(const LookupKey& other) const {
        return components == other.components;
    }
};

namespace std {
    template<>
    struct hash<LookupKey> {
        size_t operator()(const LookupKey& key) const {
            size_t result = 0;
            for (const auto& component : key.components) {
                result ^= std::hash<float>{}(component) + 0x9e3779b9 + (result << 6) + (result >> 2);
            }
            return result;
        }
    };
}

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
        FisherInformationManager(rclcpp::Node::SharedPtr node);

        FisherInformationManager();

        ~FisherInformationManager();

//////////////////////////////////////////////////////////

        /**
         * {min, max, step, total values}
         * x {0.0, 5.0, 0.1, 50}
         * y {-5 * root(3), 5 * root(3), 0.1, 174}
         * z {-5 * root(3), 5 * root(3), 0.1, 174}
         * total values in lookup: 50 * 174 * 174
         * = 1513800
         * total_size: 1513800 * 4 values per entry * 4 bytes
         * = 24,220,800
         * ~ 24 MB
         */
        void generateLookupTable(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float step);

        void loadLookupTable();

        float getInformationFromLookup(Eigen::Vector3f& landmark_camera_frame, float step);

        bool isPoseSafe(geometry_msgs::msg::Point point_from,  geometry_msgs::msg::Point point_to);
    private:
        // void callServices();
        // void checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes, std::vector<int>& changedKFIds);
        // void computeAndPopulateFIMs(std::shared_ptr<slam_msgs::srv::GetMap_Response> response);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr client_node_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        rclcpp::Client<slam_msgs::srv::GetLandmarksInView>::SharedPtr client_;
        std::unordered_map<int32_t, geometry_msgs::msg::PoseStamped> keyframe_poses_cache_;
        std::unordered_map<std::pair<Frontier, Frontier>, float, frontierPairHash> fisher_information_map_;
        std::unordered_map<LookupKey, float> lookup_table_fi_;
        // std::shared_ptr<RosVisualizer> rosVisualizer_;
    };
}

#endif // FISHER_INFORMATION_MANAGER_HPPs