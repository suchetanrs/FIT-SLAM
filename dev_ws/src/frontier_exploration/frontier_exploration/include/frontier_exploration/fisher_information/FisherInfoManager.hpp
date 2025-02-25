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
#include "frontier_exploration/util/event_logger.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

const double DISTANCE3D_THRESHOLD_KF_CHANGE = 0.1; // in m
const double ANGLESUM_THRESHOLD_KF_CHANGE = 0.6; // in rad
const float step_min = 0.09;
const float step_max = 0.3; // 0.3, 0.09
const float division_size = 0.1;
const float divisions = step_min / step_max; // the number of divisions is 10. This value is 0.1
// const float subSampleVoxelUntil_m = (1 / divisions) * division_size; // We try to fit 10 divisions in 1.0m, 11 divisions in 1.1m and so on. Every division takes up 10 cm.
const float subSampleVoxelUntil_m = -1.0;
struct LookupKey {
    std::array<float, 3> components;

    bool operator==(const LookupKey& other) const {
        return components == other.components;
    }
};

struct LookupValue {
    double information;
    int pointCount;
    unsigned int version;  // Tracks the last query this voxel was updated

    LookupValue() : information(0.0), pointCount(0), version(std::numeric_limits<unsigned int>::max()) {}
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
        std::size_t operator() (const std::pair<FrontierPtr, FrontierPtr> &pair) const {
            auto hash1 = std::hash<size_t>{}(pair.first->getUID());
            auto hash2 = std::hash<size_t>{}(pair.second->getUID());
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
        void generateLookupTable(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

        void loadLookupTable();

        float getInformationFromLookup(Eigen::Vector3f& landmark_camera_frame, float step_max, float step_min, unsigned int latest_version);

        float getInformationFromLookup(tf2::Vector3& landmark_camera_frame, float step, unsigned int latest_version);

        inline float getFactorFromNum(int num, float s)
        {
            // return std::exp(1 - std::pow(num, std::min(z / 10, 1.0f)));
            return std::exp(1 - std::pow(num, s));
        };

        inline bool getVoxelCoordinate(float& voxel_x, float& voxel_y, float& voxel_z, float landmark_x, float landmark_y, float landmark_z)
        {
            double corrected_step;
            if(abs(landmark_x) < subSampleVoxelUntil_m && abs(landmark_y) < subSampleVoxelUntil_m && abs(landmark_z) < subSampleVoxelUntil_m)
                // corrected_step = std::min(std::ceil(landmark_x / divisions) * step_min, step_max);
                corrected_step = step_min;
            else
                corrected_step = step_max;
            // std::cout << "x: " << landmark_x << " Corrected step: " << corrected_step << std::endl;
            if(corrected_step == 0)
                return false;
            voxel_x = std::round(landmark_x * (1 / corrected_step)) * corrected_step;
            voxel_y = std::round(landmark_y * (1 / corrected_step)) * corrected_step;
            voxel_z = std::round(landmark_z * (1 / corrected_step)) * corrected_step;
            return true;
        }

        bool isPoseSafe(geometry_msgs::msg::Pose& given_pose, bool exhaustiveSearch, float& information);

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
        std::unordered_map<std::pair<FrontierPtr, FrontierPtr>, float, frontierPairHash> fisher_information_map_;
        std::unordered_map<LookupKey, LookupValue> lookup_table_fi_;
        unsigned int latest_version_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fim_pointcloud_pub_;
        pcl::PointCloud<pcl::PointXYZI> fi_pointcloud_pcl_;
        unsigned int occupied_voxel_count_ = 0;
        // std::shared_ptr<RosVisualizer> rosVisualizer_;
    };
}

#endif // FISHER_INFORMATION_MANAGER_HPPs