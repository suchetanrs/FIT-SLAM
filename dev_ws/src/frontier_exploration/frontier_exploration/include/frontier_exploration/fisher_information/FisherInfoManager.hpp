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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

const double DISTANCE3D_THRESHOLD_KF_CHANGE = 0.1; // in m
const double ANGLESUM_THRESHOLD_KF_CHANGE = 0.6; // in rad

struct LookupKey {
    std::array<float, 3> components;

    bool operator==(const LookupKey& other) const {
        return components == other.components;
    }
};

struct LookupValue {
    double information;
    int pointCount;
    int version;  // Tracks the last query this voxel was updated

    LookupValue() : information(0.0), pointCount(0), version(0) {}
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

        float getInformationFromLookup(Eigen::Vector3f& landmark_camera_frame, float step, unsigned int latest_version);

        float getInformationFromLookup(tf2::Vector3& landmark_camera_frame, float step, unsigned int latest_version);

        inline float getFactorFromNum(int num, float z)
        {
            return std::exp(1 - std::pow(num, std::min(z / 10, 1.0f)));
        };

        bool isPoseSafe(geometry_msgs::msg::Pose& given_pose, bool exhaustiveSearch);

        bool isPoseSafe(geometry_msgs::msg::Point point_from,  geometry_msgs::msg::Point point_to, bool exhaustiveSearch);
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
        std::unordered_map<LookupKey, LookupValue> lookup_table_fi_;
        unsigned int latest_version_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fim_pointcloud_pub_;
        pcl::PointCloud<pcl::PointXYZI> fi_pointcloud_pcl_;
        // std::shared_ptr<RosVisualizer> rosVisualizer_;
    };
}

#endif // FISHER_INFORMATION_MANAGER_HPPs