#include <vector>
#include <algorithm>
#include <limits>
#include "frontier_exploration/Frontier.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <frontier_exploration/GeometryUtils.hpp>
#include <frontier_exploration/planners/FrontierRoadmap.hpp>

const double ARRIVAL_INFORMATION_THRESHOLD = 70.0;
const double NUM_FRONTIERS_IN_LOCAL_AREA = 3.0;
const double DISTANCE_THRESHOLD_GLOBAL_CLUSTER = 5.0;
const double CLUSTER_PADDING = 0.25;
const double LOCAL_FRONTIER_SEARCH_RADIUS = 10.0; // in m

namespace frontier_exploration
{
    struct FrontierPair {
        // Constructor
        FrontierPair(Frontier f1_, Frontier f2_) : f1(f1_), f2(f2_) {}

        Frontier f1;
        Frontier f2;

        // Custom operator< for ordering points in the map
        bool operator<(const FrontierPair& other) const {
            // First compare x coordinates, then compare y coordinates
            return (f1.getUID() + f2.getUID() < other.f1.getUID() + other.f2.getUID());
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
        FullPathOptimizer(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
                          std::shared_ptr<FrontierRoadMap> roadmap_ptr);

        // new
        void addToMarkerArrayLinePolygon(visualization_msgs::msg::MarkerArray &marker_array, std::vector<Frontier>& frontier_list,
                              std::string ns, float r, float g, float b, int id);

        void addToMarkerArraySolidPolygon(visualization_msgs::msg::MarkerArray &marker_array, geometry_msgs::msg::Point center, double radius, std::string ns, float r, float g, float b, int id);

        double calculatePathLength(std::vector<Frontier> &path);

        void getFilteredFrontiers(std::vector<Frontier> &frontier_list, size_t n, SortedFrontiers& sortedFrontiers);

        void publishLocalSearchArea(std::vector<Frontier> &frontier_list, size_t n, geometry_msgs::msg::PoseStamped& robotP);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_search_area_publisher_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::shared_ptr<FrontierRoadMap> roadmap_ptr_;
        std::map<FrontierPair, RoadmapPlanResult> frontier_pair_distances_;
    };
}