#ifndef FRONTIER_SELECTION_NODE_HPP
#define FRONTIER_SELECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <rtabmap_msgs/srv/get_nodes_in_radius.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include "octomap/octomap.h"
#include "octomap/OcTreeStamped.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include <frontier_exploration/planner.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <algorithm>
#include <fstream>
#include <thread>

#include "rtabmap_msgs/srv/get_map2.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
namespace frontier_exploration {

/**
 * STRUCTS
*/
struct PathWithInfo {
    nav_msgs::msg::Path path;
    double information_total;
};

struct SelectionResult {
    frontier_msgs::msg::Frontier frontier;
    geometry_msgs::msg::Quaternion orientation;
    bool success;
    std::map<frontier_msgs::msg::Frontier, double> frontier_costs;
};


/**
 * RAY TRACER CLASS
*/

// Add the cells which are currently unexplored to the cells_ function.
class RayTracedCells
{
    public:
        RayTracedCells(
            const nav2_costmap_2d::Costmap2D & costmap,
            std::vector<nav2_costmap_2d::MapLocation> & cells)
        : costmap_(costmap), cells_(cells)
        {
            hit_obstacle = false;
        }

        // just push the relevant cells back onto the list
        inline void operator()(unsigned int offset)
        {
            nav2_costmap_2d::MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            bool presentflag = false;
            for (auto item : cells_) {
                if(item.x == loc.x && item.y == loc.y)
                    presentflag = true;
            }
            if(presentflag == false) {
                if((int)costmap_.getCost(offset) == 255 && hit_obstacle == false) {
                    cells_.push_back(loc);
                }
                if((int)costmap_.getCost(offset) > 240 && (int)costmap_.getCost(offset) != 255) {
                    hit_obstacle = true;
                }
            }
        }

        std::vector<nav2_costmap_2d::MapLocation> getCells() {
            return cells_;
        }

    private:
        const nav2_costmap_2d::Costmap2D & costmap_;
        std::vector<nav2_costmap_2d::MapLocation> & cells_;
        bool hit_obstacle;
};

/**
 * INFORMATION CALCULATION
*/

class FrontierWithArrivalInformation {
    public:
        FrontierWithArrivalInformation(frontier_msgs::msg::Frontier frontier, int information, double alpha, int count_index, int path_length) 
        {
            frontier_ = frontier;
            information_ = information;
            alpha_ = alpha;
            count_index_ = count_index;
            path_length_ = path_length;
        }

        // Define the less-than operator for comparing instances of FrontierWithArrivalInformation
        bool operator<(const FrontierWithArrivalInformation& other) const {
            // Compare based on some criteria, e.g., information
            return path_length_ < other.path_length_;
        }

        frontier_msgs::msg::Frontier frontier_;
        size_t information_;
        double alpha_;
        int count_index_;
        int path_length_;
};

class FrontierU1Comparator {
    public:
        bool operator()(const std::pair<FrontierWithArrivalInformation, double>& a, const std::pair<FrontierWithArrivalInformation, double>& b) const {
            return a.second < b.second;
        }
};


/**
 * FRONTIER SELECTION
*/

class FrontierSelectionNode {
public:
    FrontierSelectionNode(rclcpp_lifecycle::LifecycleNode::SharedPtr node, nav2_costmap_2d::Costmap2D* costmap);

    inline int sign(int x) // get sign of an int
    {
        return x > 0 ? 1.0 : -1.0;
    }

    // void mapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

    void bresenham2D(RayTracedCells at, unsigned int abs_da, unsigned int abs_db, int error_b,
        int offset_a,
        int offset_b, unsigned int offset,
        unsigned int max_length,
        int resolution_cut_factor);
        
    SelectionResult selectFrontierCountUnknowns(
        const std::list<frontier_msgs::msg::Frontier>& frontier_list,
        std::vector<double> polygon_xy_min_max,
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
        geometry_msgs::msg::Point start_point_w, 
        std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data,
        nav2_costmap_2d::Costmap2D* traversability_costmap);

    std::pair<frontier_msgs::msg::Frontier, bool> selectFrontierClosest(
        const std::list<frontier_msgs::msg::Frontier>& frontier_list,
        const std::vector<std::vector<double>>& every_frontier,
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
        std::string globalFrameID);

    std::pair<frontier_msgs::msg::Frontier, bool> selectFrontierRandom(
        const std::list<frontier_msgs::msg::Frontier>& frontier_list,
        const std::vector<std::vector<double>>& every_frontier,
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
        std::string globalFrameID);

    std::pair<frontier_msgs::msg::Frontier, bool> selectFrontierInformationOnPath(
        const std::list<frontier_msgs::msg::Frontier>& frontier_list,
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
        geometry_msgs::msg::Point start_pose,
        std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data
    );

    
    void visualizeFrontier(const std::list<frontier_msgs::msg::Frontier>& frontier_list, 
    const std::vector<std::vector<double>>& every_frontier, std::string globalFrameID);

    void exportMapCoverage(std::vector<double> polygon_xy_min_max, std::chrono::_V2::system_clock::time_point startTime);

    // should be modified to frontier. Currently takes points.
    std::pair<PathWithInfo, bool> getPlanForFrontier(geometry_msgs::msg::Point start_point_w, frontier_msgs::msg::Frontier goal_point_w, 
    std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data, bool compute_information);

private:
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_frontier_cloud_pub;
    // viz
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_array_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr viz_pose_publisher_;

    rclcpp::Client<rtabmap_msgs::srv::GetNodesInRadius>::SharedPtr get_nodes_in_radius_client_;
    nav2_costmap_2d::Costmap2D* costmap_;
    nav2_costmap_2d::Costmap2D* traversability_costmap_;
    rclcpp::Node::SharedPtr client_node_;
    std::string mode_;
    int counter_value_;
    double frontierDetectRadius_;
    rclcpp::Logger logger_ = rclcpp::get_logger("frontier_selection");
    
    double alpha_;
    double beta_;
    std::vector<frontier_msgs::msg::Frontier> frontier_blacklist_;
    bool planner_allow_unknown_;
    int N_best_for_u2_;
};

} // namespace frontier_exploration

#endif // FRONTIER_SELECTION_NODE_HPP

