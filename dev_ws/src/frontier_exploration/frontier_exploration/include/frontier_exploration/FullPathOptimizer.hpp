#include <vector>
#include <algorithm>
#include <limits>
#include "frontier_exploration/fisher_information/FisherInfoManager.hpp"
#include "frontier_exploration/Frontier.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <frontier_exploration/util/GeometryUtils.hpp>
#include <frontier_exploration/planners/FrontierRoadmap.hpp>

const double ARRIVAL_INFORMATION_THRESHOLD = 70.0;
const double NUM_FRONTIERS_IN_LOCAL_AREA = 5.0;
const double DISTANCE_THRESHOLD_GLOBAL_CLUSTER = 5.0;
const double CLUSTER_PADDING = 0.25;
const double LOCAL_FRONTIER_SEARCH_RADIUS = 12.0; // 6.0 in m
const bool ADD_YAW_TO_TSP = false;
const bool ADD_DISTANCE_TO_ROBOT_TO_TSP = false;

const double BLACKLISTING_CIRCLE_RADIUS = 1.7; // in m
namespace frontier_exploration
{
    enum PathSafetyStatus {
        SAFE,         // 0
        UNSAFE,       // 1
        UNDETERMINED  // 2
    };

    struct FrontierPair
    {
        // Constructor
        FrontierPair(Frontier f1_, Frontier f2_) : f1(f1_), f2(f2_) {}

        Frontier f1;
        Frontier f2;

        // Custom operator< for ordering points in the map
        bool operator<(const FrontierPair &other) const
        {
            // Compare f1 and f2 in lexicographical order
            if (f1.getUID() != other.f1.getUID())
            {
                return f1.getUID() < other.f1.getUID();
            }
            return f2.getUID() < other.f2.getUID();
        }

        bool operator==(const FrontierPair &other) const
        {
            // Compare f1 and f2 in lexicographical order
            return f2.getGoalPoint() == other.f2.getGoalPoint() && f1.getGoalPoint() == other.f1.getGoalPoint();
        }
    };

    // Custom hash function for FrontierPair
    struct FrontierPairHash
    {
        std::size_t operator()(const FrontierPair &fp) const
        {
            // Combine the hash of both Frontier objects
            std::size_t h1 = std::hash<int>{}(fp.f1.getUID());
            std::size_t h2 = std::hash<int>{}(fp.f2.getUID());

            // Hash combination technique (can vary)
            return h1 ^ (h2 << 1); // Example of hash combining
        }
    };

    struct SortedFrontiers
    {
        std::vector<Frontier> local_frontiers;
        std::vector<Frontier> global_frontiers;
        Frontier closest_global_frontier;
    };

    class FullPathOptimizer
    {
    public:
        FullPathOptimizer(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

        void publishBlacklistCircles();

        void publishBlacklistPoses();

        // new
        void addToMarkerArrayLinePolygon(visualization_msgs::msg::MarkerArray &marker_array, std::vector<Frontier> &frontier_list,
                                         std::string ns, float r, float g, float b, int id);

        void addToMarkerArraySolidPolygon(visualization_msgs::msg::MarkerArray &marker_array, geometry_msgs::msg::Point center, double radius, std::string ns, float r, float g, float b, int id);

        PathSafetyStatus isPathSafe(std::vector<Frontier>& pathToFollow);

        PathSafetyStatus isRobotPoseSafe(geometry_msgs::msg::Pose& robotPose);

        double calculateLengthRobotToGoal(const Frontier& robot, const Frontier& goal, geometry_msgs::msg::PoseStamped& robotP);

        double calculatePathLength(std::vector<Frontier> &path);

        bool getBestFullPath(SortedFrontiers& sortedFrontiers, std::vector<Frontier>& bestPath, geometry_msgs::msg::PoseStamped &robotP);

        bool prepareGlobalOptimization(SortedFrontiers& sortedFrontiers, std::vector<Frontier>& bestPath, geometry_msgs::msg::PoseStamped &robotP);

        void getFilteredFrontiersN(std::vector<Frontier> &frontier_list, size_t n, SortedFrontiers &sortedFrontiers, geometry_msgs::msg::PoseStamped &robotP);

        void getFilteredFrontiers(std::vector<Frontier> &frontier_list, SortedFrontiers &sortedFrontiers, geometry_msgs::msg::PoseStamped &robotP);

        PathSafetyStatus getNextGoal(std::vector<Frontier> &frontier_list, Frontier& nextFrontier, size_t n, geometry_msgs::msg::PoseStamped &robotP, bool use_fi);

        bool refineAndPublishPath(geometry_msgs::msg::PoseStamped& robotP, Frontier& goalFrontier);

        void clearPlanCache()
        {
            frontier_pair_distances_.clear();
            pair_path_safe_.clear();
        };

        bool isInBlacklistedRegion(const Frontier& frontier)
        {
            // verify that frontier is not in blacklisted zone
            for(auto& blacklistedZone : circularBlacklistCenters_)
            {
                if(distanceBetweenFrontiers(frontier, blacklistedZone) < BLACKLISTING_CIRCLE_RADIUS)
                {
                    // frontier.setAchievability(false);
                    return true;
                }
            }
            return false;
        };

        void blacklistFrontier(Frontier& frontier)
        {
            circularBlacklistCenters_.push_back(frontier);
        }

        void blacklistFrontier(geometry_msgs::msg::PoseStamped pose, Frontier& frontier)
        {
            auto robotYaw = quatToEuler(pose.pose.orientation)[2];
            pose.pose.position = frontier.getGoalPoint();
            pose.pose.position.x += (BLACKLISTING_CIRCLE_RADIUS * cos(robotYaw));
            pose.pose.position.y += (BLACKLISTING_CIRCLE_RADIUS * sin(robotYaw));
            robotYaw += M_PI;
            auto quat = eulerToQuat(0, 0, robotYaw);
            pose.pose.orientation = quat;
            poseBlacklists_.poses.push_back(pose.pose);
        }

        void setExhaustiveSearch(bool value)
        {
            exhaustiveLandmarkSearch_ = value;
        }

        bool getExhaustiveSearch()
        {
            return exhaustiveLandmarkSearch_;
        }

        void blacklistTestCb(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_search_area_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blacklisted_region_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blacklisted_poses_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_nav2_plan_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<FisherInformationManager> fisher_information_manager_;
        std::unordered_map<FrontierPair, RoadmapPlanResult, FrontierPairHash> frontier_pair_distances_;
        std::unordered_map<FrontierPair, bool, FrontierPairHash> pair_path_safe_;
        std::vector<Frontier> circularBlacklistCenters_;
        geometry_msgs::msg::PoseArray poseBlacklists_;
        bool blacklistNextGoal_;
        double angle_for_fov_overlap_;
        bool exhaustiveLandmarkSearch_;
    };
}